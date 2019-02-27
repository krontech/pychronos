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
#ifndef _PYCHRONOS_H
#define _PYCHRONOS_H

/* Default register map size. */
#define FPGA_MAP_SIZE   0x100

/* Naked memory mapping. */
extern Py_buffer fpga_regbuffer;
extern Py_buffer fpga_rambuffer;

/* Types */
extern PyTypeObject pychronos_arrayiter_type;
extern PyTypeObject pychronos_arrayview_type;
extern PyTypeObject pychronos_fpgamap_type;
extern PyTypeObject pychronos_frame_type;
extern PyTypeObject pychronos_pwm_type;

/* Methods */
extern PyObject *pychronos_read_raw(PyObject *self, PyObject *args);
extern PyObject *pychronos_read_frame(PyObject *self, PyObject *args);
extern PyObject *pychronos_write_frame(PyObject *self, PyObject *args);

#endif /* _PYCHRONOS_H */
