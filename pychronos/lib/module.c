/****************************************************************************
 *  Copyright (C) 2019 Kron Technologies Inc <http://www.krontech.ca>.      *
 *                                                                          *
 *  This program is free software: you can redistribute it and/or modify    *
 *  it under the terms of the GNU General Public License as published by    *
 *  the Free Software Foundation, either version 3 of the License, or       *
 *  (at your option) any later version.                                     *
 *                                                                          *
 *  This program is distributed in the hope that it will be useful,         *
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of          *
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the           *
 *  GNU General Public License for more details.                            *
 *                                                                          *
 *  You should have received a copy of the GNU General Public License       *
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.   *
 ****************************************************************************/
#include <Python.h>
#include <structmember.h>

#include <string.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>

#include "fpga.h"
#include "pychronos.h"

/* Try opening the memory device, or return -1 and set errno. */
static int
pychronos_try_maps(void)
{
    static int memfd = -1;
    if (memfd < 0) {
        memfd = open("/dev/mem", O_RDWR | O_SYNC);
    }
    return memfd;
}

/* Wrapper to perform a register mmap, or return NULL on failure. */
void *
pychronos_reg_mmap(unsigned long roffset, unsigned long rsize)
{
    static uint8_t *regs = MAP_FAILED;

    /* Map the GPMC register space. */
    if (regs == MAP_FAILED) {
        int fd = pychronos_try_maps();
        if (fd < 0) {
            PyErr_SetFromErrno(PyExc_OSError);
            return NULL;
        }
        regs = mmap(0, GPMC_REGISTER_LEN, PROT_READ | PROT_WRITE, MAP_SHARED, fd, GPMC_RANGE_BASE + GPMC_REGISTER_OFFSET);
        if (regs == MAP_FAILED) {
            PyErr_SetFromErrno(PyExc_OSError);
            return NULL;
        }
    }

    if ((rsize + roffset) > GPMC_REGISTER_LEN) {
        PyErr_SetString(PyExc_ValueError, "Register map size exceeds the GPMC register space.");
        return NULL;
    }
    return regs + roffset;
}

void *
pychronos_ram_mmap(unsigned long roffset, unsigned long rsize)
{
    static uint8_t *ram = MAP_FAILED;

    /* Map the GPMC RAM space. */
    if (ram == MAP_FAILED) {
        int fd = pychronos_try_maps();
        if (fd < 0) {
            PyErr_SetFromErrno(PyExc_OSError);
            return NULL;
        }
        ram = mmap(0, GPMC_RAM_LEN, PROT_READ | PROT_WRITE, MAP_SHARED, fd, GPMC_RANGE_BASE + GPMC_RAM_OFFSET);
        if (ram == MAP_FAILED) {
            PyErr_SetFromErrno(PyExc_OSError);
            return NULL;
        }
    }

    if ((rsize + roffset) > GPMC_RAM_LEN) {
        PyErr_SetString(PyExc_ValueError, "Register map size exceeds the GPMC RAM space.");
        return NULL;
    }
    return ram + roffset;
}

/*===============================================*
 * Generic Array Iterator
 *===============================================*/
struct pychronos_arrayiter {
    PyObject_HEAD
    PyObject *array;
    Py_ssize_t length;
    Py_ssize_t next;
};

static int
pychronos_arrayiter_init(PyObject *self, PyObject *args, PyObject *kwargs)
{
    struct pychronos_arrayiter *iter = (struct pychronos_arrayiter *)self;
    char *keywords[] = { "parent", NULL };
    PyObject *parent;
    PyTypeObject *type;

    if (!PyArg_ParseTupleAndKeywords(args, kwargs, "O", keywords, &parent)) {
        return -1;
    }
    type = Py_TYPE(parent);
    if (!type->tp_as_mapping) {
        PyErr_SetString(PyExc_TypeError, "Parent must support the mapping protocol.");
        return -1;
    }
    Py_INCREF(parent);
    iter->array = parent;
    iter->length = type->tp_as_mapping->mp_length(parent);
    iter->next = 0;
    return 0;
}

static PyObject *
pychronos_arrayiter_next(PyObject *self)
{
    struct pychronos_arrayiter *iter = (struct pychronos_arrayiter *)self;
    PyTypeObject *type = Py_TYPE(iter->array);

    if (iter->next < iter->length) {
        PyObject *ret = type->tp_as_mapping->mp_subscript(iter->array, PyLong_FromLong(iter->next));
        iter->next++;
        return ret;
    }
    else {
        PyErr_SetNone(PyExc_StopIteration);
        return NULL;
    }
}

static void
pychronos_arrayiter_dealloc(PyObject *self)
{
    struct pychronos_arrayiter *iter = (struct pychronos_arrayiter *)self;
    Py_XDECREF(iter->array);
    Py_TYPE(self)->tp_free((PyObject *)self);
}

PyTypeObject pychronos_arrayiter_type = {
    PyVarObject_HEAD_INIT(NULL, 0)
    .tp_name = "pychronos.arrayiter",
    .tp_doc = "Generic Array Iterator",
    .tp_basicsize = sizeof(struct pychronos_arrayiter),
    .tp_itemsize = 0,
    .tp_flags = Py_TPFLAGS_DEFAULT,
    .tp_new = PyType_GenericNew,
    .tp_dealloc = pychronos_arrayiter_dealloc,
    .tp_init = pychronos_arrayiter_init,
    .tp_iternext = pychronos_arrayiter_next
};

/*=====================================*
 * SPI Read and Write Helper
 *=====================================*/
static PyObject *
pychronos_write_spi(PyObject *self, PyObject *args, PyObject *kwargs)
{
    PyObject *ret = NULL;
    void *rxbuf;
    int spifd = -1;

    /* Python argument parsing. */
    char *keywords[] = {
        "device",
        "data",
        "mode",
        "csel",
        "speed",
        "bitsPerWord",
        NULL,
    };
    const char *path;
    Py_buffer data;
    unsigned char mode = 0;
    unsigned long speed = 1000000;
    unsigned int wordsize = 0;
    const char *csel = NULL;
    if (!PyArg_ParseTupleAndKeywords(args, kwargs, "sy*|bskI", keywords,
            &path, &data, &mode, &csel, &speed, &wordsize)) {
        return NULL;
    }

    rxbuf = malloc(data.len);
    if(!rxbuf) {
        PyBuffer_Release(&data);
        PyErr_NoMemory();
        return NULL;
    }
    /* Open the SPI device. */
    spifd = open(path, O_RDWR);
    if (spifd < 0) {
        PyErr_SetFromErrno(PyExc_OSError);
        PyBuffer_Release(&data);
        free(rxbuf);
        return NULL;
    }

    /* Break out to cleanup and return */
    do {
        struct spi_ioc_transfer xfer = {
            .tx_buf = (uintptr_t)data.buf,
            .rx_buf = (uintptr_t)rxbuf,
            .len = data.len,
            .speed_hz = speed,
            .delay_usecs = 0,
            .bits_per_word = wordsize,
            .cs_change = 0,
        };

        if (ioctl(spifd, SPI_IOC_WR_MODE, &mode)) {
            PyErr_SetFromErrno(PyExc_OSError);
            break;
        }
        /* Optional soft chip-select */
        if (csel) {
            int spics = open(csel, O_WRONLY);
            if (spics < 0) {
                PyErr_SetFromErrno(PyExc_OSError);
                break;
            }
            write(spics, "0", 1);
            if (ioctl(spifd, SPI_IOC_MESSAGE(1), &xfer) < 0) {
                PyErr_SetFromErrno(PyExc_OSError);
            } else {
                ret = PyBytes_FromStringAndSize(rxbuf, data.len);
            }
            write(spics, "1", 1);
            close(spics);
        }
        /* Hardware chipselect */
        else {
            if (ioctl(spifd, SPI_IOC_MESSAGE(1), &xfer) < 0) {
                PyErr_SetFromErrno(PyExc_OSError);
            } else {
                ret = PyBytes_FromStringAndSize(rxbuf, data.len);
            }
        }
    } while(0);

    /* Cleanup */
    PyBuffer_Release(&data);
    free(rxbuf);
    close(spifd);
    return ret;
}

PyDoc_STRVAR(pychronos_write_spi_docstring,
"writespi(device, data, csel=None, mode=0)\n\
--\n\
\n\
Helper function to write data to an SPI device and return the bytes\n\
received back via the MISO signal.\n\
\n\
Parameters\n\
----------\n\
device : `string`\n\
    Path to the SPI device to open for writing.\n\
data : `bytes`\n\
    Data to write to the SPI device.\n\
mode : `int`, optional\n\
    SPI clock phase and polarity (default: 0).\n\
csel : `string`, optional\n\
    Path to GPIO device to use as a soft chip select.\n\
speed : `int`, optional\n\
    SPI device speed in Hertz (default: 1000000).\n\
bitsPerWord : `int`, optional\n\
    SPI word size in bits (default: 8).");

/*=====================================*
 * Chronos Python Module
 *=====================================*/
PyDoc_STRVAR(pychronos__doc__,
"This module provides an interface to the Chronos FPGA and image sensor.\n\
Each section of the FPGA register space is represented as classes, providing\n\
read and write access to the underlying registers.");

PyDoc_STRVAR(pychronos_read_raw_docstring,
    "readraw(address, length) -> bytes\n\n"
    "Read raw data from acquisition memory.");

PyDoc_STRVAR(pychronos_read_frame_docstring,
    "readframe(address, width, height) -> array\n\n"
    "Read a frame from acquisition memory.");

PyDoc_STRVAR(pychronos_write_frame_docstring,
    "writeframe(address, buffer) -> None\n\n"
    "Write a frame to acquisition memory.");

static PyMethodDef pychronos_methods[] = {
    {"readraw",  pychronos_read_raw,        METH_VARARGS, pychronos_read_raw_docstring},
    {"readframe",  pychronos_read_frame,    METH_VARARGS, pychronos_read_frame_docstring},
    {"writeframe", pychronos_write_frame,   METH_VARARGS, pychronos_write_frame_docstring},
    {"writespi",   (PyCFunction)pychronos_write_spi, METH_VARARGS | METH_KEYWORDS, pychronos_write_spi_docstring},
    {NULL, NULL, 0, NULL}
};

static PyTypeObject *pubtypes[] = {
    &pychronos_arrayiter_type,
    &pychronos_arrayview_type,
    &pychronos_fpgamap_type,
    &pychronos_frame_type,
    &pychronos_pwm_type,
};

static void
pychronos_init_types(PyObject *mod)
{
    int i;
    
    /* Register all public types. */
    for (i = 0; i < sizeof(pubtypes)/sizeof(pubtypes[0]); i++) {
        PyTypeObject *t = pubtypes[i]; 
        if (PyType_Ready(t) == 0) {
            Py_INCREF(t);
            PyModule_AddObject(mod, strchr(t->tp_name, '.') + 1,  (PyObject *)t);
        }
    }

    /* Register macros. */
    PyModule_AddIntMacro(mod, FPGA_MAP_SIZE);
    PyModule_AddIntMacro(mod, FPGA_TIMEBASE_HZ);
    PyModule_AddIntMacro(mod, FPGA_SENSOR_BASE);
    PyModule_AddIntMacro(mod, FPGA_SEQUENCER_BASE);
    PyModule_AddIntMacro(mod, FPGA_TRIGGER_BASE);
    PyModule_AddIntMacro(mod, FPGA_SEQPROGRAM_BASE);
    PyModule_AddIntMacro(mod, FPGA_DISPLAY_BASE);
    PyModule_AddIntMacro(mod, FPGA_CONFIG_BASE);
    PyModule_AddIntMacro(mod, FPGA_COL_GAIN_BASE);
    PyModule_AddIntMacro(mod, FPGA_VRAM_BASE);
    PyModule_AddIntMacro(mod, FPGA_SCI_BASE);
    PyModule_AddIntMacro(mod, FPGA_COL_OFFSET_BASE);
    PyModule_AddIntMacro(mod, FPGA_IO_BASE);
    PyModule_AddIntMacro(mod, FPGA_TIMING_BASE);
    PyModule_AddIntMacro(mod, FPGA_IMAGER_BASE);
    PyModule_AddIntMacro(mod, FPGA_PIPELINE_BASE);
    PyModule_AddIntMacro(mod, FPGA_VIDSRC_BASE);
    PyModule_AddIntMacro(mod, FPGA_CALSRC_BASE);
    PyModule_AddIntMacro(mod, FPGA_OVERLAY_BASE);
    PyModule_AddIntMacro(mod, FPGA_COL_CURVE_BASE);
}

#if PY_MAJOR_VERSION >= 3
/*=====================================*
 * Chronos Python Module (python3)
 *=====================================*/
static PyModuleDef libpychronos_module = {
    PyModuleDef_HEAD_INIT,
    "libpychronos",     /* name of the module */
    pychronos__doc__,   /* module documentation */
    -1,                 /* module uses global state */
    pychronos_methods,  /* module methods */
};

PyMODINIT_FUNC
PyInit_libpychronos(void)
{
    PyObject *mod;

    /*
     * Try opening the memory device at module import, but fail silently
     * if it happens to be inaccessible. This will punt the error reporting
     * over to the type constructors if memory is not actually accessible.
     */
    pychronos_try_maps();

    /* Load the module. */
    mod = PyModule_Create(&libpychronos_module);
    if (mod == NULL) {
        return NULL;
    }

    pychronos_init_types(mod);
    return mod;
}
#else
/*=====================================*
 * Chronos Python Module (python2)
 *=====================================*/
PyMODINIT_FUNC
initlibpychronos(void)
{
    PyObject *mod;

    /*
     * Try opening the memory device at module import, but fail silently
     * if it happens to be inaccessible. This will punt the error reporting
     * over to the type constructors if memory is not actually accessible.
     */
    pychronos_try_maps();

    mod = Py_InitModule3("libpychronos", pychronos_methods, pychronos__doc__);
    if (mod == NULL) {
        return;
    }

    pychronos_init_types(mod);
}
#endif