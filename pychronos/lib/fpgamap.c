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
 * FPGA Memory Mapping Base Type
 *=====================================*/
struct fpgamap {
    PyObject_HEAD
    void            *regs;
    unsigned long   roffset;
    unsigned long   rsize;
};

struct fpgamap_reginfo {
    unsigned long   offset; /* Starting offset of the register. */
    unsigned long   size;   /* Length of the scalar (or of a single element in the array). */
    unsigned long   count;  /* Zero for scalar types, length of the array for array types. */
    unsigned long long mask; /* Mask of register subfield to extract, or zero for full read/write. */
};
#define FPGA_REG_INFO(_offset_, _size_, _count_) (struct fpgamap_reginfo []){{_offset_, _size_, _count_, 0}}

#define FPGA_REG_TYPED(_container_, _member_, _type_) \
    FPGA_REG_INFO(offsetof(_container_, _member_), sizeof(_type_), 0)

#define FPGA_REG_SCALAR(_container_, _member_) \
    FPGA_REG_INFO(offsetof(_container_, _member_), sizeof(((_container_ *)0)->_member_), 0)

#define FPGA_REG_ARRAY(_container_, _member_) \
    FPGA_REG_INFO(offsetof(_container_, _member_), sizeof(((_container_ *)0)->_member_[0]), arraysize(((_container_ *)0)->_member_))

#define fpgaptr(_self_, _offset_, _type_) \
    (_type_ *)((unsigned char *)(((struct fpgamap *)_self_)->regs) + (uintptr_t)(_offset_))

/*=====================================*
 * Register Accessor Methods
 *=====================================*/
static PyObject *
fpga_get_uint(PyObject *self, void *closure)
{
    struct fpgamap_reginfo *info = closure;
    unsigned long long value = 0;
    unsigned long long lsb;
    switch (info->size) {
        case sizeof(uint8_t):
            value = *fpgaptr(self, info->offset, uint8_t);
            break;
        case sizeof(uint16_t):
            value = *fpgaptr(self, info->offset, uint16_t);
            break;
        case sizeof(uint32_t):
            value = *fpgaptr(self, info->offset, uint32_t);
            break;
        case sizeof(uint64_t):
            value = *fpgaptr(self, info->offset, uint64_t);
            break;
        default:
            PyErr_SetString(PyExc_RuntimeError, "Invalid register access size");
            return NULL;
    }
    
    /* Simple read. */
    if (info->mask == 0) {
        return PyLong_FromUnsignedLongLong(value);
    }

    /* Perform a masked read. */
    lsb = (~info->mask + 1) & info->mask;
    if (info->mask != lsb) {
        return PyLong_FromUnsignedLongLong((value & info->mask) / lsb);
    }
    else if (value & info->mask) {
        Py_RETURN_TRUE;
    }
    else {
        Py_RETURN_FALSE;
    }
}

static int
fpga_rmw_uint(PyObject *self, PyObject *value, void *closure)
{
    struct fpgamap_reginfo *info = closure;
    unsigned long long lsb = (~info->mask + 1) & info->mask;
    unsigned long long intval, prev;
    PyObject *intobj;

    /* Convert whatever we got into an integer. */
    intobj = PyNumber_Long(value);
    if (!intobj) {
        return -1;
    }
    intval = PyLong_AsUnsignedLongLong(intobj) * lsb;
    Py_DECREF(intobj);
    
    /* Ensure that it fits within the mask. */
    if ((info->mask >> (info->size * 8)) != 0) {
        PyErr_SetString(PyExc_ValueError, "Register mask out of range for size.");
        return -1;
    }
    if ((intval & ~info->mask) != 0) {
        PyErr_SetString(PyExc_ValueError, "Register value out of range");
        return -1;
    }

    switch (info->size) {
        case sizeof(uint8_t):
            prev = *fpgaptr(self, info->offset, uint8_t) & ~info->mask;
            *fpgaptr(self, info->offset, uint8_t) = (prev | intval);
            break;
        case sizeof(uint16_t):
            prev = *fpgaptr(self, info->offset, uint16_t) & ~info->mask;
            *fpgaptr(self, info->offset, uint16_t) = (prev | intval);
            break;
        case sizeof(uint32_t):
            prev = *fpgaptr(self, info->offset, uint32_t) & ~info->mask;
            *fpgaptr(self, info->offset, uint32_t) = (prev | intval);
            break;
        case sizeof(uint64_t):
            prev = *fpgaptr(self, info->offset, uint64_t) & ~info->mask;
            *fpgaptr(self, info->offset, uint64_t) = (prev | intval);
            break;
        default:
            PyErr_SetString(PyExc_RuntimeError, "Invalid register access size");
            return -1;
    }
    return 0;
}

static int
fpga_set_uint(PyObject *self, PyObject *value, void *closure)
{
    struct fpgamap_reginfo *info = closure;
    unsigned long long intval;
    PyObject *intobj;

    /* Special case for masked writes. */
    if (info->mask != 0) {
        return fpga_rmw_uint(self, value, closure);
    }

    /* Convert whatever we got into an integer. */
    intobj = PyNumber_Long(value);
    if (!intobj) {
        return -1;
    }
    intval = PyLong_AsUnsignedLongLong(intobj);
    Py_DECREF(intobj);

    /* Sanity check the range. */
    if ((intval >> (info->size * 8)) != 0) {
        Py_DECREF(intobj);
        PyErr_SetString(PyExc_ValueError, "Register value out of range");
        return -1;
    }

    /* Write it into the registers. */
    switch (info->size) {
        case sizeof(uint8_t):
            *fpgaptr(self, info->offset, uint8_t) = intval;
            break;
        case sizeof(uint16_t):
            *fpgaptr(self, info->offset, uint16_t) = intval;
            break;
        case sizeof(uint32_t):
            *fpgaptr(self, info->offset, uint32_t) = intval;
            break;
        case sizeof(uint64_t):
            *fpgaptr(self, info->offset, uint64_t) = intval;
            break;
        default:
            PyErr_SetString(PyExc_RuntimeError, "Invalid register access size");
            return -1;
    }
    return 0;
}

/*=====================================*
 * Array and Memory View Methods
 *=====================================*/
struct fpgamap_arrayview {
    PyObject_HEAD
    PyObject        *parent;
    unsigned long   offset;
    unsigned long   itemsize;
    Py_ssize_t      itemcount;
};

static Py_ssize_t
arrayview_length(PyObject *self)
{
    struct fpgamap_arrayview *view = (struct fpgamap_arrayview *)self;
    return view->itemcount;
}

static PyObject *
arrayview_getval(PyObject *self, PyObject *key)
{
    struct fpgamap_arrayview *view = (struct fpgamap_arrayview *)self;
    unsigned long index = PyLong_AsUnsignedLong(key);
    struct fpgamap_reginfo info = {
        .offset = view->offset + (index * view->itemsize),
        .size = view->itemsize,
        .count = view->itemcount,
        .mask = 0,
    };

    /* Sanity check the index. */
    if (PyErr_Occurred()) {
        return NULL;
    }
    if (index >= view->itemcount) {
        PyErr_SetString(PyExc_IndexError, "Register index out of range");
        return NULL;
    }

    return fpga_get_uint(view->parent, &info);
}

static int
arrayview_setval(PyObject *self, PyObject *key, PyObject *value)
{
    struct fpgamap_arrayview *view = (struct fpgamap_arrayview *)self;
    unsigned long index = PyLong_AsUnsignedLong(key);
    struct fpgamap_reginfo info = {
        .offset = view->offset + (index * view->itemsize),
        .size = view->itemsize,
        .count = view->itemcount,
        .mask = 0,
    };

    /* Sanity check the index. */
    if (PyErr_Occurred()) {
        return -1;
    }
    if (index >= view->itemcount) {
        PyErr_SetString(PyExc_IndexError, "Register index out of range");
        return -1;
    }

    return fpga_set_uint(view->parent, value, &info);
}

static int
arrayview_getbuffer(PyObject *self, Py_buffer *buffer, int flags)
{
    struct fpgamap_arrayview *view = (struct fpgamap_arrayview *)self;

    buffer->buf = fpgaptr(view->parent, view->offset, void);
    buffer->obj = self;
    buffer->len = view->itemcount * view->itemsize;
    buffer->readonly = 0;
    buffer->itemsize = view->itemsize;
    switch (view->itemsize) {
        case 1:
            buffer->format = (flags & PyBUF_FORMAT) ? "B" : NULL;
            break;
        case 2:
            buffer->format = (flags & PyBUF_FORMAT) ? "H" : NULL;
            break;
        case 4:
            buffer->format = (flags & PyBUF_FORMAT) ? "I" : NULL;
            break;
        case 8:
            buffer->format = (flags & PyBUF_FORMAT) ? "Q" : NULL;
            break;
        default:
            PyErr_SetString(PyExc_RuntimeError, "Invalid register access size");
            return -1;
    }
    buffer->ndim = 1;
    buffer->shape = (flags & PyBUF_ND) ? &view->itemcount : NULL;
    /* Simple n-dimensional array */
    buffer->strides = NULL;
    buffer->suboffsets = NULL;
    buffer->internal = NULL;

    Py_INCREF(self);
    return 0;
}

PyDoc_STRVAR(fpgamap_arrayview_docstring,
"arrayview(parent, offset=0, size=4, count=FPGA_MAP_SIZE/size)\n\
--\n\
\n\
Return an array view for a subset of the FPGA register space, start\n\
offset bytes into the parent fpgamap class.\n\
\n\
Parameters\n\
----------\n\
parent : `fpgamap`\n\
    The parent FPGA register mapping class.\n\
offset : `int`, optional\n\
    Starting offset of the array into the parent mapping (zero, by default)\n\
size : `int`, optional\n\
    Length of a single member of the array (4, default)\n\
count : `int`\n\
    Number of array elements to export (FPGA_MAP_SIZE/size, by default)");

static int
arrayview_init(PyObject *self, PyObject *args, PyObject *kwargs)
{
    struct fpgamap_arrayview *view = (struct fpgamap_arrayview *)self;
    struct fpgamap *fmap;
    PyObject *parent;
    char *keywords[] = {
        "parent",
        "offset",
        "size",
        "count",
        NULL,
    };

    view->parent = NULL;
    view->offset = 0;
    view->itemsize = 4;
    view->itemcount = 0;
    if (!PyArg_ParseTupleAndKeywords(args, kwargs, "O|kkn", keywords,
            &parent, &view->offset, &view->itemsize, &view->itemcount)) {
        return -1;
    }
    
    if (PyType_IsSubtype(Py_TYPE(parent), &pychronos_fpgamap_type) != 1) {
        PyErr_SetString(PyExc_TypeError, "Parent must be an subtype of fpgamap");
        return -1;
    }
    fmap = (struct fpgamap *)parent;
    if (view->offset + (view->itemcount * view->itemsize) > fmap->rsize) {
        /* The array view doesn't fit into the parent mapping. */
        PyErr_SetString(PyExc_ValueError, "Array mapping exceeds parent address space");
        return -1;
    }
    /* If no count was specified, infer it from the parent mapping. */
    if (view->itemcount == 0) {
        view->itemcount = (fmap->rsize - view->offset) / view->itemsize;
    }

    /* Success */
    view->parent = parent;
    Py_INCREF(view->parent);
    return 0;
}

/* Arrayview destructor */
static void
arrayview_dealloc(PyObject *self)
{
    struct fpgamap_arrayview *view = (struct fpgamap_arrayview *)self;
    if (view->parent) {
        Py_DECREF(view->parent);
        view->parent = NULL;
    }
    Py_TYPE(self)->tp_free(self);
}

static PyMappingMethods arrayview_as_array = {
    .mp_length = arrayview_length,
    .mp_subscript = arrayview_getval,
    .mp_ass_subscript = arrayview_setval
};

static PyBufferProcs arrayview_as_buffer = {
    .bf_getbuffer = arrayview_getbuffer,
};

static PyObject *
arrayview_getiter(PyObject *self)
{
    return PyObject_CallFunction((PyObject *)&pychronos_arrayiter_type, "O", self);
}

PyTypeObject pychronos_arrayview_type = {
    PyVarObject_HEAD_INIT(NULL, 0)
    .tp_name = "pychronos.arrayview",
    .tp_doc = fpgamap_arrayview_docstring,
    .tp_basicsize = sizeof(struct fpgamap_arrayview),
    .tp_itemsize = 0,
    .tp_flags = Py_TPFLAGS_DEFAULT,
    .tp_new = PyType_GenericNew,
    .tp_init = arrayview_init,
    .tp_dealloc = arrayview_dealloc,
    .tp_as_mapping = &arrayview_as_array,
    .tp_as_buffer = &arrayview_as_buffer,
    .tp_iter = arrayview_getiter,
};

/* Getter to return an arrayview object. */
static PyObject *
fpga_get_arrayview(PyObject *self, void *closure)
{
    struct fpgamap_reginfo *info = closure;
    PyObject *args = Py_BuildValue("(O,k,k,n)", self, info->offset, info->size, info->count);
    if (args) {
        PyObject *buf = PyObject_CallObject((PyObject *)&pychronos_arrayview_type, args);
        Py_DECREF(args);
        return buf;
    }
    return NULL;
}

/* MemoryView Protocol. */
static int
fpgamap_getbuffer(PyObject *self, Py_buffer *view, int flags)
{
    struct fpgamap *fmap = (struct fpgamap *)self;
    view->format = (flags & PyBUF_FORMAT) ? "<H" : NULL;
    return PyBuffer_FillInfo(view, self, fmap->regs, fmap->rsize, 0, flags);
}
static PyBufferProcs fpgamap_as_buffer = { .bf_getbuffer = fpgamap_getbuffer };

static PyGetSetDef fpgamap_getset[] = {
    {"mem8",  fpga_get_arrayview, NULL, "8-bit memory array view",  FPGA_REG_INFO(0, sizeof(uint8_t), 0) },
    {"mem16", fpga_get_arrayview, NULL, "16-bit memory array view", FPGA_REG_INFO(0, sizeof(uint16_t), 0) },
    {"mem32", fpga_get_arrayview, NULL, "32-bit memory array view", FPGA_REG_INFO(0, sizeof(uint32_t), 0) },
    { NULL },
};

PyDoc_STRVAR(fpgamap_reg_read_docstring,
"regRead(offset, size)\n\
--\n\
\n\
Helper function to read an integer value of the given size from memory\n\
beginning offset bytes from the start of the register mapping.\n\
\n\
Parameters\n\
----------\n\
offset : `int`\n\
    Offset in bytes from the start of the register mapping.\n\
size : `int`\n\
    Length of the integer to read.\n\
mask : `int`, optional\n\
    Mask of bits to extract from the register value.\n\
\n\
Returns\n\
-------\n\
int");

static PyObject *
fpgamap_reg_read(PyObject *self, PyObject *args, PyObject *kwargs)
{
    struct fpgamap_reginfo info;
    char *keywords[] = {
        "offset",
        "size",
        "mask",
        NULL,
    };
    info.mask = 0;
    info.count = 1;
    if (!PyArg_ParseTupleAndKeywords(args, kwargs, "kk|K", keywords, &info.offset, &info.size, &info.mask)) {
        return NULL;
    }
    return fpga_get_uint(self, &info);
}

PyDoc_STRVAR(fpgamap_reg_write_docstring,
"regWrite(offset, size, value)\n\
--\n\
\n\
Helper function to write an integer value of the given size into memory\n\
at the offset bytes from the start of the register mapping.\n\
\n\
Parameters\n\
----------\n\
offset : `int`\n\
    Offset in bytes from the start of the register mapping.\n\
size : `int`\n\
    Length of the integer to write.\n\
value : `object`\n\
    Value to be written, must be convertable to an integer.\n\
mask : `int`, optional\n\
    Mask of bits to write into the register value.");

static PyObject *
fpgamap_reg_write(PyObject *self, PyObject *args, PyObject *kwargs)
{
    struct fpgamap_reginfo info;
    PyObject *value;
    char *keywords[] = {
        "offset",
        "size",
        "value",
        "mask",
        NULL,
    };
    info.count = 1;
    info.mask = 0;
    if (!PyArg_ParseTupleAndKeywords(args, kwargs, "kkO|K", keywords, &info.offset, &info.size, &value, &info.mask)) {
        return NULL;
    }
    fpga_set_uint(self, value, &info);
    Py_RETURN_NONE;
}

PyDoc_STRVAR(fpgamap_reg_array_docstring,
"regArray(offset, size, count)\n\
--\n\
\n\
Helper function to generate an array view from a subset of the register\n\
mapping, starting at a given offset into memory. This function is a wrapper\n\
for the expression: `arrayview(self, offset, size, count)`\n\
\n\
Parameters\n\
----------\n\
offset : `int`\n\
    Offset in bytes from the start of the register mapping.\n\
size : `int`\n\
    Size of the array elements.\n\
count : `int`\n\
    Number of array elements.");

static PyObject *
fpgamap_reg_array(PyObject *self, PyObject *args, PyObject *kwargs)
{
    struct fpgamap_reginfo info;
    char *keywords[] = {
        "offset",
        "size",
        "count",
        NULL,
    };
    if (!PyArg_ParseTupleAndKeywords(args, kwargs, "|kkk", keywords, &info.offset, &info.size, &info.count)) {
        return NULL;
    }
    return fpga_get_arrayview(self, &info);
}

PyDoc_STRVAR(fpgamap_reg_atomic_docstring,
"regAtomic(offset, size)\n\
--\n\
\n\
Helper function to perform an atomic read of an integer value of the\n\
given size from memory beginning offset bytes from the start of the\n\
register mapping.\n\
\n\
Since the memory bus to the FPGA is only 16-bits, it is possible that\n\
reading larger integers may be preempted by the FPGA internals, leading\n\
to corrupted values. This method reads a stable value by double-reading\n\
until we get the same value twice.\n\
\n\
Parameters\n\
----------\n\
offset : `int`\n\
    Offset in bytes from the start of the register mapping.\n\
size : `int`\n\
    Length of the integer to read.\n\
\n\
Returns\n\
-------\n\
int");
static PyObject *
fpgamap_reg_atomic(PyObject *self, PyObject *args, PyObject *kwargs)
{
    unsigned long offset;
    unsigned long size;
    char *keywords[] = {
        "offset",
        "size",
        NULL,
    };
    if (!PyArg_ParseTupleAndKeywords(args, kwargs, "kk", keywords, &offset, &size)) {
        return NULL;
    }

    if (size == 4) {
        volatile uint32_t *ptr = fpgaptr(self, offset, volatile uint32_t);
        uint32_t value = 0;
        for (;;) {
            uint32_t test = *ptr;
            value = *ptr;
            if (test == value) break;
        }
        return PyLong_FromUnsignedLongLong(value);
    }
    else if (size == 8) {
        volatile uint64_t *ptr = fpgaptr(self, offset, volatile uint64_t);
        uint64_t value = 0;
        for (;;) {
            uint64_t test = *ptr;
            value = *ptr;
            if (test == value) break;
        }
        return PyLong_FromUnsignedLongLong(value);
    }
    /* Otherwise, this is the wrong size for an atomic access */
    else {
        PyErr_SetString(PyExc_RuntimeError, "Invalid atomic register access size");
        return NULL;
    }
}

static PyMethodDef fpgamap_methods[] = {
    {"regRead",     (PyCFunction)fpgamap_reg_read,  METH_VARARGS | METH_KEYWORDS, fpgamap_reg_read_docstring},
    {"regWrite",    (PyCFunction)fpgamap_reg_write, METH_VARARGS | METH_KEYWORDS, fpgamap_reg_write_docstring},
    {"regArray",    (PyCFunction)fpgamap_reg_array, METH_VARARGS | METH_KEYWORDS, fpgamap_reg_array_docstring},
    {"regAtomic",   (PyCFunction)fpgamap_reg_atomic, METH_VARARGS | METH_KEYWORDS, fpgamap_reg_atomic_docstring},
    {NULL, NULL, 0, NULL},
};

PyDoc_STRVAR(fpgamap_docstring,
"fpgamap(offset=0, size=FPGA_MAP_SIZE)\n\
--\n\
\n\
Return a new map of the FPGA register space starting at offset, and\n\
covering size bytes of the address space. These maps form the base\n\
class for all other FPGA register blocks and provide raw access to\n\
memory.\n\
\n\
Parameters\n\
----------\n\
offset : `int`, optional\n\
    Starting offset of the register mapping (zero, by default)\n\
size : `int`, optional\n\
    Length of the register mapping (FPGA_MAP_SIZE, by default)");

static int
fpgamap_init(PyObject *self, PyObject *args, PyObject *kwargs)
{
    struct fpgamap *fmap = (struct fpgamap *)self;
    char *keywords[] = {
        "offset",
        "size",
        NULL,
    };
    if (!PyArg_ParseTupleAndKeywords(args, kwargs, "|kk", keywords, &fmap->roffset, &fmap->rsize)) {
        return -1;
    }
    fmap->regs = pychronos_reg_mmap(fmap->roffset, fmap->rsize);
    if (!fmap->regs) {
        return -1;
    }
    return 0;
}

static PyObject *
fpgamap_new(PyTypeObject *type, PyObject *args, PyObject *kwds)
{
    struct fpgamap *self = (struct fpgamap *)type->tp_alloc(type, 0);
    self->roffset = 0;
    self->rsize = FPGA_MAP_SIZE;
    self->regs = NULL;
    return (PyObject *)self;
}

PyTypeObject pychronos_fpgamap_type = {
    PyVarObject_HEAD_INIT(NULL, 0)
    .tp_name = "pychronos.fpgamap",
    .tp_doc = fpgamap_docstring,
    .tp_basicsize = sizeof(struct fpgamap),
    .tp_itemsize = 0,
    .tp_flags = Py_TPFLAGS_DEFAULT | Py_TPFLAGS_BASETYPE,
    .tp_init = fpgamap_init,
    .tp_new = fpgamap_new,
    .tp_methods = fpgamap_methods,
    .tp_getset = fpgamap_getset,
    .tp_as_buffer = &fpgamap_as_buffer,
};
