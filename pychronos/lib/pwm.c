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
#include "pychronos.h"

/*=====================================*
 * PWM Register Definitions
 *=====================================*/
/* TI Davinci timer peripheral registers. */
struct pwm_dm814x {
    const uint32_t  tidr;       /* Identification Register */
    uint32_t __reserved0[3];
    uint32_t tiocp_cfg;         /* Timer OCP Configuration Register */
    uint32_t __reserved1[3];
    uint32_t irq_eoi;           /* Timer IRQ End-of-Interrupt register */
    const uint32_t irqstatus_raw;
    const uint32_t irqstatus;   /* Timer IRQSTATUS register */
    uint32_t irqenable_set;     /* Timer IRQENABLE set register */
    uint32_t irqenable_clr;     /* Timer IRQENABLE clear register */
    uint32_t irqwaken;          /* Timer IRQ Wakeup Enable Register */
    uint32_t tclr;              /* Timer Control Register */
    uint32_t tcrr;              /* Timer Counter Register */
    uint32_t tldr;              /* Timer Load Register  */
    uint32_t ttgr;              /* Timer Trigger Register */
    const uint32_t twps;        /* Timer Write Posted Status Register */
    uint32_t tmar;              /* Timer match Register */
    uint32_t tcar1;             /* Timer Capture Register 1 */
    uint32_t tsicr;             /* Timer Synchronous Interface Control Register */
    uint32_t tcar2;             /* Timer Capture Register 2 */
};

/* TIOCP_CFG Register: This register allows controlling various parameters of the OCP interface. */
#define TIM_TIOCP_SOFTRESET         (1 << 0)
#define TIM_TIOCP_EMUFREE           (1 << 1)
#define TIM_TIOCP_IDLEMODE          (0x3 << 2)
#define TIM_TIOCP_IDLEMODE_FORCE_IDLE   (0 << 2)
#define TIM_TIOCP_IDLEMODE_NO_IDLE      (1 << 2)
#define TIM_TIOCP_IDLEMODE_SMART_IDLE   (2 << 2)
#define TIM_TIOCP_IDLEMODE_SMART_WKUP   (3 << 2)

/* IRQ_EOI Register: Software End-of-Interrupt allows generation of pulses on the interrupt line. */
#define TIM_IRQ_EOI_LINE_NUMBER (1 << 0)

/* IRQSTATUS, IRQENABLE and IRQWAKEN Registers */
#define TIM_IRQ_TCAR_FLAG       (1 << 2)    /* IRQ status for capture */
#define TIM_IRQ_OVF_FLAG        (1 << 1)    /* IRQ status for overflow */
#define TIM_IRQ_MAT_FLAG        (1 << 0)    /* IRQ status for match */

/* TCLR Register */
#define TIM_TCLR_GPO_CFG        (1 << 14)   /* General purpose output - directly drives the PORGPOCFG pin */
#define TIM_TCLR_CAPT_MODE      (1 << 13)   /* Capture mode  */
#define TIM_TCLR_PT             (1 << 12)   /* Pulse or toggle mode on PORTIMERPWM output pin */
#define TIM_TCLR_TRG            (0x3 << 10) /* Trigger output mode on PORTIMERPWM output pin */
#define TIM_TCLR_TRG_NONE           (0 << 10)   /* Trigger disabled. */
#define TIM_TCLR_TRG_OVF            (1 << 10)   /* Trigger on overflow */
#define TIM_TCLR_TRG_MAT            (2 << 10)   /* Trigger on overflow and match */
#define TIM_TCLR_TCM            (0x3 << 8)  /* Transition Capture Mode on PIEVENTCAPT inpiut pin */
#define TIM_TCLR_TCM_NONE           (0 << 8)    /* No capture */
#define TIM_TCLR_TCM_RISING         (1 << 8)    /* Capture on low to high transition */
#define TIM_TCLR_TCM_FALLING        (2 << 8)    /* Capture on high to low transition */
#define TIM_TCLR_TCM_BOTH           (3 << 8)    /* Capture on both edge transition */
#define TIM_TCLR_SCPWM          (1 << 7)    /* This bit should be set or cleared while the timer trigger is stopped or the trigger is off */
#define TIM_TCLR_CE             (1 << 6)    /* Compare mode */
#define TIM_TCLR_PRE            (1 << 5)    /* Prescaler enable */
#define TIM_TCLR_PTV            (0x7 << 2)  /* Pre-scale clock timer value. */
#define TIM_TCLR_PTV_2              (0 << 2)
#define TIM_TCLR_PTV_4              (1 << 2)
#define TIM_TCLR_PTV_8              (2 << 2)
#define TIM_TCLR_PTV_16             (3 << 2)
#define TIM_TCLR_PTV_32             (4 << 2)
#define TIM_TCLR_PTV_64             (5 << 2)
#define TIM_TCLR_PTV_128            (6 << 2)
#define TIM_TCLR_PTV_256            (7 << 2)
#define TIM_TCLR_AR             (1 << 1)    /* Auto-reload timer */
#define TIM_TCLR_ST             (1 << 0)    /* Start timer */
/* Get the effective clock divider from the TCLR register. */
#define TIM_TCLR_DIV(_reg_)     (((_reg_) & TIM_TCLR_PRE) ? (2 << (((_reg_) & TIM_TCLR_PTV) >> 2)) : 1)

/* TWPS Register */
#define TIM_TWPS_W_PEND_TMAR    (1 << 4)
#define TIM_TWPS_W_PEND_TTGR    (1 << 3)
#define TIM_TWPS_W_PEND_TLDR    (1 << 2)
#define TIM_TWPS_W_PEND_TCRR    (1 << 1)
#define TIM_TWPS_W_PEND_TCLR    (1 << 0)

/* TSICR Register */
#define TIM_TSICR_POSTED        (1 << 2)    /* Posted mode active */
#define TIM_TSICR_SFT           (1 << 1)    /* Soft reset */

/* Some misc constants */
#define TIM_DEVOSC_FREQ         20000000    /* Default clock source is the 20MHz oscillator. */
#define TIM_MAP_SIZE            4096

/* Timer peripheral addresses - as mapped on the Chronos 1.4. */
const uintptr_t pwm_timer_addrs[] = {
    0, /* Skip IO0 */
    0x48048000, /* TIMER6 */
    0x4804A000, /* TIMER7 */
};
const unsigned int pwm_timer_count = sizeof(pwm_timer_addrs)/sizeof(pwm_timer_addrs[0]);

struct pwm_object {
    PyObject_HEAD
    int     mapfd;
    void    *mapregs;
    double        duty;
    unsigned long clkduty;      /* PWM duty cycle in clocks. */
    unsigned long clkperiod;    /* PWM Period in clocks */
};

static int
pwm_update(struct pwm_object *pwm, double duty, unsigned long freq)
{
    volatile struct pwm_dm814x *regs = pwm->mapregs;

    /* Validate the duty cycle and frequency, and convert them into clock periods. */
    if ((duty < 0.0) || (duty > 1.0)) {
        PyErr_SetString(PyExc_ValueError, "Invalid PWM duty cycle");
        return -1;
    }
    if (freq == 0) {
        PyErr_SetString(PyExc_ValueError, "Invalid PWM timer frequency");
        return -1;
    }
    if (freq > (TIM_DEVOSC_FREQ/256)) {
        PyErr_SetString(PyExc_ValueError, "PWM timer frequency too fast");
        return -1;
    }
    pwm->duty = duty;
    pwm->clkperiod = (TIM_DEVOSC_FREQ  + freq - 1) / freq;
    pwm->clkduty = pwm->clkperiod * pwm->duty;
    if (pwm->clkduty > (pwm->clkperiod-2)) pwm->clkduty = pwm->clkperiod-2;
    else if (pwm->clkduty < 2) pwm->clkduty = 2;

    /* Update the registers. */
    regs->tldr = 0xffffffff - pwm->clkperiod;
    regs->tmar = 0xffffffff - pwm->clkduty;
    return 0;
}

static int
pwm_init(PyObject *self, PyObject *args, PyObject *kwargs)
{
    struct pwm_object *pwm = (struct pwm_object *)self;
    volatile struct pwm_dm814x *regs;
    char *keywords[] = {
        "id",
        "freq",
        "duty",
        NULL,
    };
    unsigned int    id;
    unsigned long   freq;
    uintptr_t       addr;

    pwm->duty = 0.5;
    if (!PyArg_ParseTupleAndKeywords(args, kwargs, "Ik|d", keywords, &id, &freq, &pwm->duty)) {
        return -1;
    }

    /* Lookup the PWM peripheral to use. */
    if ((id >= pwm_timer_count) || !pwm_timer_addrs[id]) {
        PyErr_Format(PyExc_ValueError, "PWM peripheral %u not found", id);
        return -1;
    }
    addr = pwm_timer_addrs[id];

    /* Open memory mapping to the PWM */
    pwm->mapfd = open("/dev/mem", O_RDWR | O_SYNC);
    if (pwm->mapfd < 0) {
        PyErr_SetFromErrno(PyExc_OSError);
        return -1;
    }
    pwm->mapregs = mmap(0, TIM_MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, pwm->mapfd, addr);
    if (pwm->mapregs == MAP_FAILED) {
        PyErr_SetFromErrno(PyExc_OSError);
        close(pwm->mapfd);
        pwm->mapregs = NULL;
        return -1;
    }
    regs = pwm->mapregs;

    /* Configure the PWM timer */
    regs->tclr =  TIM_TCLR_PT | TIM_TCLR_TRG_MAT | TIM_TCLR_SCPWM | TIM_TCLR_CE | TIM_TCLR_AR;
    if (pwm_update(pwm, pwm->duty, freq)) {
        close(pwm->mapfd);
        munmap(pwm->mapregs, TIM_MAP_SIZE);
        pwm->mapregs = NULL;
        return -1;
    }

    /* Start the PWM timer. */
    regs->tclr |= TIM_TCLR_ST;
    regs->ttgr = 0;
    return 0;
}

static void
pwm_dealloc(PyObject *self)
{
    struct pwm_object *pwm = (struct pwm_object *)self;
    if (pwm->mapregs) {
        munmap(pwm->mapregs, TIM_MAP_SIZE);
        close(pwm->mapfd);
        pwm->mapregs = NULL;
    }
    Py_TYPE(self)->tp_free((PyObject *)self);
}

static PyObject *
pwm_get_freq(PyObject *self, void *closure)
{
    struct pwm_object *pwm = (struct pwm_object *)self;
    return PyLong_FromUnsignedLong(TIM_DEVOSC_FREQ / pwm->clkperiod);
}

static int
pwm_set_freq(PyObject *self, PyObject *value, void *closure)
{
    struct pwm_object *pwm = (struct pwm_object *)self;
    unsigned long freq;

    freq = PyLong_AsUnsignedLong(value);
    if (PyErr_Occurred()) {
        return -1;
    }
    return pwm_update(pwm, pwm->duty, freq);
}

static PyObject *
pwm_get_duty(PyObject *self, void *closure)
{
    struct pwm_object *pwm = (struct pwm_object *)self;
    return PyFloat_FromDouble((double)pwm->clkduty / (double)pwm->clkperiod);
}

static int
pwm_set_duty(PyObject *self, PyObject *value, void *closure)
{
    struct pwm_object *pwm = (struct pwm_object *)self;
    double duty = PyFloat_AsDouble(value);
    if (PyErr_Occurred()) {
        return -1;
    }
    return pwm_update(pwm, duty, TIM_DEVOSC_FREQ / pwm->clkperiod);
}

static PyGetSetDef pwm_getset[] = {
    {"freq", pwm_get_freq, pwm_set_freq, "PWM frequency",  NULL },
    {"duty", pwm_get_duty, pwm_set_duty, "PWM duty cycle", NULL },
    { NULL },
};

PyDoc_STRVAR(pwm_docstring,
"pwm(id, freq, duty=0.5)\n\
--\n\
\n\
Create an object to control a hardware PWM peripheral.\n\
\n\
Parameters\n\
----------\n\
id : `int`\n\
    PWM timer id.\n\
freq : `int`\n\
    PWM timer period, in hertz.\n\
duty : `float`, optional\n\
    Initial duty cycle in the range of 0.0 to 1.0 (default: 0.5)");

PyTypeObject pychronos_pwm_type = {
    PyVarObject_HEAD_INIT(NULL, 0)
    .tp_name = "pychronos.pwm",
    .tp_doc = pwm_docstring,
    .tp_basicsize = sizeof(struct pwm_object),
    .tp_itemsize = 0,
    .tp_flags = Py_TPFLAGS_DEFAULT,
    .tp_new = PyType_GenericNew,
    .tp_init = pwm_init,
    .tp_dealloc = pwm_dealloc,
    .tp_getset = pwm_getset,
};
