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
#include "fpga.h"
#include "pychronos.h"

/*=====================================*
 * Frame Object Type
 *=====================================*/
struct frame_object {
    PyObject_VAR_HEAD
    Py_ssize_t vRes;
    Py_ssize_t hRes;
    uint16_t data[];
};

static Py_ssize_t
frame_array_length(PyObject *self)
{
    struct frame_object *frame = (struct frame_object *)self;
    return (frame->hRes * frame->vRes);
}

static PyObject *
frame_array_getval(PyObject *self, PyObject *key)
{
    struct frame_object *frame = (struct frame_object *)self;
    unsigned long index = PyLong_AsUnsignedLong(key);

    if (PyErr_Occurred()) {
        return NULL;
    }
    if (index >= (frame->hRes * frame->vRes)) {
        PyErr_SetString(PyExc_IndexError, "Array index out of range");
        return NULL;
    }
    return PyLong_FromLong(frame->data[index]);
}

static int
frame_array_setval(PyObject *self, PyObject *key, PyObject *val)
{
    struct frame_object *frame = (struct frame_object *)self;
    unsigned long index, value;

    /* Parse the register address. */
    index = PyLong_AsUnsignedLong(key);
    if (PyErr_Occurred()) {
        return -1;
    }
    if (index >= (frame->hRes * frame->vRes)) {
        PyErr_SetString(PyExc_IndexError, "Array index out of range");
        return -1;
    }

    /* Parse the register value. */
    value = PyLong_AsUnsignedLong(val);
    if (PyErr_Occurred()) {
        return -1;
    }
    if (value >= 0xfff) {
        PyErr_SetString(PyExc_ValueError, "Register value out of range");
        return -1;
    }
    frame->data[index] = value;
    return 0;
}

static PyMappingMethods frame_as_array = {
    .mp_length = frame_array_length,
    .mp_subscript = frame_array_getval,
    .mp_ass_subscript = frame_array_setval
};

static int
frame_buffer_getbuffer(PyObject *self, Py_buffer *view, int flags)
{
    struct frame_object *frame = (struct frame_object *)self;

    view->buf = frame->data;
    view->obj = self;
    view->len = frame->hRes * frame->vRes * sizeof(uint16_t);
    view->readonly = 0;
    view->itemsize = sizeof(uint16_t);
    view->format = (flags & PyBUF_FORMAT) ? "H" : NULL;
    if (flags & PyBUF_ND) {
        view->ndim = 2;
        view->shape = &frame->vRes; /* Superhacky */
    } else {
        view->ndim = 1;
        view->shape = NULL;
    }
    /* Simple 2-dimensional array */
    view->strides = NULL;
    view->suboffsets = NULL;
    view->internal = NULL;

    Py_INCREF(self);
    return 0;
}

static PyBufferProcs frame_as_buffer = {
    .bf_getbuffer = frame_buffer_getbuffer,
    .bf_releasebuffer = NULL,
};

static PyObject *
frame_getiter(PyObject *self)
{
    return PyObject_CallFunction((PyObject *)&pychronos_arrayiter_type, "O", self);
}

static PyObject *
frame_new(PyTypeObject *type, PyObject *args, PyObject *kwds)
{
    struct frame_object *self;
    unsigned long hRes, vRes;

    if (!PyArg_ParseTuple(args, "ll", &hRes, &vRes)) {
        return NULL;
    }
    self = (struct frame_object *)type->tp_alloc(type, hRes * vRes);
    if (self) {
        self->hRes = hRes;
        self->vRes = vRes;
    }
    return (PyObject *)self;
}

static PyMemberDef frame_members[] = {
    { "hRes", T_PYSSIZET, offsetof(struct frame_object, hRes), READONLY, "Horizontal Resolution" },
    { "vRes", T_PYSSIZET, offsetof(struct frame_object, vRes), READONLY, "Vertical Resolution" },
    { NULL }
};

PyDoc_STRVAR(frame_docstring,
"frame(hRes, vRes)\n\
--\n\
\n\
Create a frame buffer to contain an image of the desired resolution\n\
and initially set to zeroes. The frame can be accessed as a 2D array\n\
of integers.\n\
\n\
Parameters\n\
----------\n\
hRes, vRes : `int`\n\
    Horizontal and vertical resolution of the frame");

PyTypeObject pychronos_frame_type = {
    PyVarObject_HEAD_INIT(NULL, 0)
    .tp_name = "pychronos.frame",
    .tp_doc = frame_docstring,
    .tp_basicsize = sizeof(struct frame_object),
    .tp_itemsize = sizeof(uint16_t),
    .tp_flags = Py_TPFLAGS_DEFAULT,
    .tp_new = frame_new,
    .tp_as_buffer = &frame_as_buffer,
    .tp_as_mapping = &frame_as_array,
    .tp_iter = frame_getiter,
    .tp_members = frame_members,
};

/*=====================================*
 * Fast RAM Readout Helper
 *=====================================*/
static void *
pychronos_read_alloc(volatile struct fpga_vram *vram, unsigned long address, unsigned long length)
{
    /* Acquire some scratch memory for the readout. */
    uint8_t *buffer = malloc(length);
    if (!buffer) {
        PyErr_NoMemory();
        return NULL;
    }

    /* Can we do readout via the fast region? */
    if (vram->identifier == VRAM_IDENTIFIER) {
        uint32_t offset = 0;
        int i;
    
        vram->burst = 0x20;
        for (offset = 0; offset < length; offset += sizeof(vram->buffer)) {
            /* Read a page from memory into the VRAM buffer. */
            vram->address = address + (offset / FPGA_FRAME_WORD_SIZE);
            vram->control = VRAM_CTL_TRIG_READ;
            for (i = 0; i < 1000; i++) {
                if (vram->control == 0) break;
            }

            /* Copy memory out of the VRAM buffer. */
            if ((offset + sizeof(vram->buffer)) >= length) {
                memcpy(buffer + offset, (void *)vram->buffer, length - offset);
            } else {
                memcpy(buffer + offset, (void *)vram->buffer, sizeof(vram->buffer));
            }
        }
    }
    /* Readout via the old method. */
    else {
        uint32_t *pageoffset = (uint32_t *)(fpga_regbuffer.buf + FPGA_PAGE_OFFSET_BASE);
        *pageoffset = address;
        memcpy(buffer, fpga_rambuffer.buf, length);
        *pageoffset = 0;
    }

    return buffer;
}

static void
pychronos_write_ram(volatile struct fpga_vram *vram, unsigned long address, const uint8_t *data, unsigned long len)
{
    /* If the fast VRAM block is unavailable, use the slow interface */
    if (vram->identifier != VRAM_IDENTIFIER) {
        uint32_t *pageoffset = (uint32_t *)(fpga_regbuffer.buf + FPGA_PAGE_OFFSET_BASE);
        *pageoffset = address;
        memcpy(fpga_rambuffer.buf, data, len);
        *pageoffset = 0;
    }
    /* Otherwise, write the data in bursts via the VRAM block. */
    else {
        uint32_t offset = 0;
        int i;
    
        vram->burst = 0x20;
        for (offset = 0; offset < len; offset += sizeof(vram->buffer)) {
            /* Copy memory into the VRAM buffer. */
            if ((offset + sizeof(vram->buffer)) >= len) {
                memcpy((void *)vram->buffer, data + offset, len - offset);
            } else {
                memcpy((void *)vram->buffer, data + offset, sizeof(vram->buffer));
            }

            /* Write the page into memory using the fast VRAM block. */
            vram->address = address + (offset / FPGA_FRAME_WORD_SIZE);
            vram->control = VRAM_CTL_TRIG_WRITE;
            for (i = 0; i < 1000; i++) {
                if (vram->control == 0) break;
            }
        }
    }
}

PyObject *
pychronos_read_raw(PyObject *self, PyObject *args)
{
    struct fpga_vram *vram = (struct fpga_vram *)(fpga_regbuffer.buf + FPGA_VRAM_BASE);
    unsigned long address, length;
    uint8_t *buffer;
    PyObject *obj;

    if (!PyArg_ParseTuple(args, "ll", &address, &length)) {
        return NULL;
    }

    buffer = pychronos_read_alloc(vram, address, length);
    if (!buffer) {
        return NULL;
    }

    obj = Py_BuildValue("y#", buffer, (int)length);
    free(buffer);
    return obj;
}

PyObject *
pychronos_read_frame(PyObject *self, PyObject *args)
{
    struct fpga_vram *vram = (struct fpga_vram *)(fpga_regbuffer.buf + FPGA_VRAM_BASE);
    struct frame_object *frame;
    unsigned long address, length;
    uint8_t *buffer;
    uint16_t *pixels;
    int ndims[2];
    int i;

    if (!PyArg_ParseTuple(args, "lii", &address, &ndims[1], &ndims[0])) {
        return NULL;
    }
    if ((ndims[0] <= 0) || (ndims[1] <= 0)) {
        PyErr_SetString(PyExc_ValueError, "Resolution out of range");
        return NULL;
    }

    /* Read the raw packed frame out of RAM. */
    length = (ndims[0] * ndims[1] * 12) / 8;
    buffer = pychronos_read_alloc(vram, address, length);
    if (!buffer) {
        return NULL;
    }

    frame = PyObject_NewVar(struct frame_object, &pychronos_frame_type, ndims[0] * ndims[1]);
    if (!frame) {
        free(buffer);
        return NULL;
    }
    frame->hRes = ndims[1];
    frame->vRes = ndims[0];
    pixels = frame->data;

#ifdef __ARM_NEON
    for (i = 0; (i + 24) <= length; i += 24) {
        asm volatile (
            "   vld3.8 {d0,d1,d2}, [%[s]]   \n"
            /* low nibble of split byte to high nibble of first pixel. */
            "   vshll.u8  q2, d1, #8        \n"
            "   vaddw.u8  q2, q2, d0        \n"
            "   vand.u16  q2, #0x0fff       \n" /* q2 = first pixel */
            /* high nibble of split byte to low nibble of second pixel. */
            "   vshr.u8   d1, d1, #4        \n"
            "   vshll.u8  q3, d2, #4        \n"
            "   vaddw.u8  q3, q3, d1        \n" /* q3 = second pixel */
            /* write out */
            "   vst2.16 {q2,q3}, [%[d]]     \n"
            :: [d]"r"(pixels), [s]"r"(buffer + i) : );
        pixels += 16;
    }
#else
    for (i = 0; (i+3) <= length; i += 3) {
        uint8_t split = buffer[i + 1];
        *(pixels++) = (buffer[i + 0] << 0) | (split & 0x0f) << 8;
        *(pixels++) = (buffer[i + 2] << 4) | (split & 0xf0) >> 4;
    }
#endif

    free(buffer);
    return (PyObject *)frame;
}

PyObject *
pychronos_write_frame(PyObject *self, PyObject *args)
{
    struct fpga_vram *vram = (struct fpga_vram *)(fpga_regbuffer.buf + FPGA_VRAM_BASE);
    unsigned long address;
    PyObject *frame;
    Py_buffer pbuffer;

    if (!PyArg_ParseTuple(args, "lO", &address, &frame)) {
        return NULL;
    }
    if (PyObject_GetBuffer(frame, &pbuffer, PyBUF_FORMAT | PyBUF_ND) != 0) {
        return NULL;
    }

    /* Handle by array item size. */
    if (pbuffer.itemsize == 1) {
        /* Assume a raw frame already packed into 12-bit data. */
        pychronos_write_ram(vram, address, pbuffer.buf, pbuffer.len);
    }
    /* Convert uint16 array into 12-bit packed frame data. */
    else if (pbuffer.itemsize == 2) {
        int i;

        /* Allocate memory for the packed representation */
        Py_ssize_t packsize = (pbuffer.len * 12) / (pbuffer.itemsize * 8);
        uint16_t *pixels = pbuffer.buf;
        uint8_t *packed = malloc(packsize);
        if (!packed) {
            PyErr_NoMemory();
            return NULL;
        }

#ifdef __ARM_NEON
        for (i = 0; (i + 24) <= packsize; i += 24) {
            asm volatile (
                "   vld2.16 {q0,q1}, [%[s]] \n" /* q0 = first pixel, q1 = second pixel */
                "   vmovn.u16 d4, q0        \n" /* d4 = low 8-lsb of first pixel */
                "   vshrn.u16 d6, q1, #4    \n" /* d6 = high 8-msb of second pixel */
                /* Combine the split byte */
                "   vshrn.u16 d5, q0, #8    \n" /* d5 = high nibble of first pixel */
                "   vmovn.u16 d0, q1        \n" /* d0 = low 8-msb of second pixel */
                "   vsli.8    d5, d0, #4    \n" /* d5 = split byte */
                /* write out */
                "   vst3.8 {d4,d5,d6}, [%[d]]   \n"
                :: [d]"r"(packed + i), [s]"r"(pixels) : );
            pixels += 16;
        }
#else
        for (i = 0; (i+3) <= packsize; i += 3) {
            uint16_t first = *(pixels++);
            uint16_t second = *(pixels++);
            packed[i + 0] = ((first >> 0) & 0xff);
            packed[i + 1] = ((first >> 8) & 0xf0) | ((second >> 0) & 0xf);
            packed[i + 2] = ((second >> 4) & 0xff);
        }
#endif
        pychronos_write_ram(vram, address, packed, packsize);
        free(packed);
    }
    /* TODO: Convert uint32 and floating point types. */
    else {
        PyErr_SetString(PyExc_NotImplementedError, "Unable to convert buffer with itemsize > 2");
        PyBuffer_Release(&pbuffer);
        return NULL;
    }

    PyBuffer_Release(&pbuffer);
    Py_RETURN_NONE;
}
