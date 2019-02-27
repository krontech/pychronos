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

Py_buffer fpga_regbuffer = {
    .buf = MAP_FAILED,
    .obj = NULL,
    .len = GPMC_REGISTER_LEN,
    .format = "H",
    .readonly = 0,
    .itemsize = sizeof(uint16_t),
    .ndim = 1,
    .shape = (Py_ssize_t[]){GPMC_REGISTER_LEN / sizeof(uint16_t)},
    .strides = (Py_ssize_t[]){sizeof(uint16_t)},
};

Py_buffer fpga_rambuffer = {
    .buf = MAP_FAILED,
    .obj = NULL,
    .len = GPMC_RAM_LEN,
    .format = "H",
    .readonly = 0,
    .itemsize = sizeof(uint16_t),
    .ndim = 1,
    .shape = (Py_ssize_t[]){GPMC_RAM_LEN / sizeof(uint16_t)},
    .strides = (Py_ssize_t[]){sizeof(uint16_t)},
};

int
pychronos_init_maps(void)
{
    static int fd = -1;
    if (fd >= 0) return 0;

    fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd < 0) {
        PyErr_SetFromErrno(PyExc_OSError);
        return -1;
    }

    fpga_regbuffer.buf = mmap(0, GPMC_REGISTER_LEN, PROT_READ | PROT_WRITE, MAP_SHARED, fd, GPMC_RANGE_BASE + GPMC_REGISTER_OFFSET);
    if (fpga_regbuffer.buf == MAP_FAILED) {
        PyErr_SetFromErrno(PyExc_OSError);
        close(fd);
        fd = -1;
        return -1;
    }

    fpga_rambuffer.buf = mmap(0, GPMC_RAM_LEN, PROT_READ | PROT_WRITE, MAP_SHARED, fd, GPMC_RANGE_BASE + GPMC_RAM_OFFSET);
    if (fpga_rambuffer.buf == MAP_FAILED) {
        PyErr_SetFromErrno(PyExc_OSError);
        munmap(fpga_regbuffer.buf, GPMC_REGISTER_LEN);
        close(fd);
        fd = -1;
        return -1;
    }

    /* Success */
    return 0;
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
pychronos_arrayiter_finalize(PyObject *self)
{
    struct pychronos_arrayiter *iter = (struct pychronos_arrayiter *)self;
    Py_DECREF(iter->array);
}

PyTypeObject pychronos_arrayiter_type = {
    PyVarObject_HEAD_INIT(NULL, 0)
    .tp_name = "pychronos.arrayiter",
    .tp_doc = "Generic Array Iterator",
    .tp_basicsize = sizeof(struct pychronos_arrayiter),
    .tp_itemsize = 0,
    .tp_flags = Py_TPFLAGS_DEFAULT | Py_TPFLAGS_HAVE_FINALIZE,
    .tp_new = PyType_GenericNew,
    .tp_init = pychronos_arrayiter_init,
    .tp_iternext = pychronos_arrayiter_next,
    .tp_finalize = pychronos_arrayiter_finalize,
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
    int i;
    PyObject *mod, *o;
    PyTypeObject *pubtypes[] = {
        &pychronos_arrayiter_type,
        &pychronos_arrayview_type,
        &pychronos_fpgamap_type,
        &pychronos_frame_type,
    };

    /* Initialize the FPGA register mapping and the Python objects. */
    if (pychronos_init_maps()) {
        return NULL;
    }

    /* Load the module. */
    mod = PyModule_Create(&libpychronos_module);
    if (mod == NULL) {
        return NULL;
    }
    PyModule_AddIntMacro(mod, FPGA_MAP_SIZE);

    /* Register the raw memory mapping */
    o = PyMemoryView_FromBuffer(&fpga_regbuffer);
    if (o == NULL) {
        return NULL;
    }
    Py_INCREF(o);
    PyModule_AddObject(mod, "reg",  o);

    o = PyMemoryView_FromBuffer(&fpga_regbuffer);
    if (o == NULL) {
        return NULL;
    }
    Py_INCREF(o);
    PyModule_AddObject(mod, "ram",  o);

    /* Register all public types. */
    for (i = 0; i < sizeof(pubtypes)/sizeof(pubtypes[0]); i++) {
        PyTypeObject *t = pubtypes[i]; 
        if (PyType_Ready(t) < 0) {
            return NULL;
        }
        Py_INCREF(t);
        PyModule_AddObject(mod, strchr(t->tp_name, '.') + 1,  (PyObject *)t);
    }

    PyType_Ready(&pychronos_arrayiter_type);
    Py_INCREF(&pychronos_arrayiter_type);
    return mod;
}
