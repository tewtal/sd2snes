/*----------------------------------------------------------------------------
 *      U S B  -  K e r n e l
 *----------------------------------------------------------------------------
 * Name:    usbuser.c
 * Purpose: USB Custom User Module
 * Version: V1.20
 *----------------------------------------------------------------------------
 *      This software is supplied "AS IS" without any warranties, express,
 *      implied or statutory, including but not limited to the implied
 *      warranties of fitness for purpose, satisfactory quality and
 *      noninfringement. Keil extends you a royalty-free right to reproduce
 *      and distribute executable files created using this software for use
 *      on NXP Semiconductors LPC family microcontroller devices only. Nothing
 *      else gives you the right to use this software.
 *
 * Copyright (c) 2009 Keil - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/
#include <stdint.h>

//debug
#include <stdio.h>
//debug

#include "config.h"
#include "usb.h"
#include "usbcfg.h"
#include "usbhw.h"
#include "usbcore.h"
#include "usbuser.h"
#include "cdcuser.h"
#include "usb2p.h"
#include "timer.h"   /* getticks(), HZ — SysTick fallback for usb2p_hires_cycles */

extern unsigned char soft_bulkin_int;

/*
 *  USB Power Event Callback
 *   Called automatically on USB Power Event
 *    Parameter:       power: On(TRUE)/Off(FALSE)
 */

#if USB_POWER_EVENT
void USB_Power_Event (uint32_t  power) {
}
#endif


/*
 *  USB Reset Event Callback
 *   Called automatically on USB Reset Event
 */

#if USB_RESET_EVENT
void USB_Reset_Event (void) {
  CDC2P_Disable();
  usb2p_reset();
  USB_ResetCore();
}
#endif


/*
 *  USB Suspend Event Callback
 *   Called automatically on USB Suspend Event
 */

#if USB_SUSPEND_EVENT
void USB_Suspend_Event (void) {
}
#endif


/*
 *  USB Resume Event Callback
 *   Called automatically on USB Resume Event
 */

#if USB_RESUME_EVENT
void USB_Resume_Event (void) {
}
#endif


/*
 *  USB Remote Wakeup Event Callback
 *   Called automatically on USB Remote Wakeup Event
 */

#if USB_WAKEUP_EVENT
void USB_WakeUp_Event (void) {
}
#endif


/*
 *  USB Start of Frame Event Callback
 *   Called automatically on USB Start of Frame Event
 */

#if USB_SOF_EVENT
void USB_SOF_Event (void) {
}
#endif


/*
 *  USB Error Event Callback
 *   Called automatically on USB Error Event
 *    Parameter:       error: Error Code
 */

#if USB_ERROR_EVENT
void USB_Error_Event (uint32_t error) {
}
#endif


/*
 *  USB Set Configuration Event Callback
 *   Called automatically on USB Set Configuration Request
 */

#if USB_CONFIGURE_EVENT
void USB_Configure_Event (void) {

  if (USB_Configuration) {                  /* Check if USB is configured */
    /* add your code here */
    //saturnu new - org: not there:>
	CDC_block_conf ();
    CDC2P_Disable();
    usb2p_reset();
  }
}
#endif


/*
 *  USB Set Interface Event Callback
 *   Called automatically on USB Set Interface Request
 */

#if USB_INTERFACE_EVENT
void USB_Interface_Event (void) {
}
#endif


/*
 *  USB Set/Clear Feature Event Callback
 *   Called automatically on USB Set/Clear Feature Request
 */

#if USB_FEATURE_EVENT
void USB_Feature_Event (void) {
}
#endif


#define P_EP(n) ((USB_EP_EVENT & (1 << (n))) ? USB_EndPoint##n : (void*)0)

/* USB Endpoint Events Callback Pointers */
void (* const USB_P_EP[16]) (uint32_t event) = {
  P_EP(0),
  P_EP(1),
  P_EP(2),
  P_EP(3),
  P_EP(4),
  P_EP(5),
  P_EP(6),
  P_EP(7),
  P_EP(8),
  P_EP(9),
  P_EP(10),
  P_EP(11),
  P_EP(12),
  P_EP(13),
  P_EP(14),
  P_EP(15),
};


/*
 *  USB Endpoint 1 Event Callback
 *   Called automatically on USB Endpoint 1 Event
 *    Parameter:       event
 */

void USB_EndPoint1 (uint32_t event) {
  uint16_t temp;
  static uint16_t serialState;

  switch (event) {
    case USB_EVT_IN:
      temp = CDC_GetSerialState();
      if (serialState != temp) {
         serialState = temp;
         CDC_NotificationIn();            /* send SERIAL_STATE notification */
         // printf("CDC_NotificationIn  %x\n", temp);
      }
      break;
  }
}


/*
 *  USB Endpoint 2 Event Callback
 *   Called automatically on USB Endpoint 2 Event
 *    Parameter:       event
 */

void USB_EndPoint2 (uint32_t event) {

  switch (event) {
    case USB_EVT_OUT:
      CDC_BulkOut ();                /* data received from Host */
      break;
    case USB_EVT_IN:
   // printf("USB_EVT_IN %x %x\n", (uint16_t)(event>>16), (uint16_t)event);
	//if(soft_bulkin_int){
      CDC_BulkIn ();                 /* data expected from Host */
	//soft_bulkin_int=0;
	//}
      break;
  }
}


/*
 *  USB Endpoint 3 Event Callback
 *   Called automatically on USB Endpoint 3 Event
 *    Parameter:       event
 */

void USB_EndPoint3 (uint32_t event) {
}


/*
 *  USB Endpoint 4 Event Callback
 *   Called automatically on USB Endpoint 4 Event
 *    Parameter:       event
 */

void USB_EndPoint4 (uint32_t event) {
}

static uint8_t usb2p_usb_irq_enabled(void) {
#if defined(USB_IRQn)
  return NVIC_GetEnableIRQ(USB_IRQn) != 0;
#elif defined(OTG_FS_IRQn)
  return NVIC_GetEnableIRQ(OTG_FS_IRQn) != 0;
#else
  return 1;
#endif
}

/* Short critical section guarding tx_buf shared state between the main-loop
   producer (usb2p_queue_frame) and the USB-ISR consumer (usb2p_tx_peek via
   CDC2P_KickTx).  Masks only the USB IRQ — the same interrupt the ISR
   runs on — so the window is just the tx_buf pointer/byte manipulation, NOT
   the slow SPI read that precedes it.  This is what lets IN-complete IRQs
   keep shipping buffered packets while the main loop reads the next DAT chunk
   over SPI (the SPI read runs with the USB IRQ enabled).

   Save/restore semantics: usb2p_queue_frame runs in BOTH the main loop (drain)
   and the USB ISR itself (queuing a HELLO/INFO/NAK response).  In the ISR the
   USB IRQ is already masked, so we must only re-enable on unlock if it was
   enabled on lock — otherwise we'd wrongly enable the IRQ from inside the ISR.
   Returns the prior state to pass back to unlock. */
uint8_t usb2p_tx_lock(void) {
  uint8_t was_enabled = usb2p_usb_irq_enabled();
  if (was_enabled) USB_DisableIRQ();
  return was_enabled;
}
void usb2p_tx_unlock(uint8_t prev) {
  if (prev) USB_EnableIRQ();
}

/* Free-running high-resolution clock for sub-tick watch polling.
 *
 * The 100 Hz SysTick is too coarse to schedule address-watch polls finely
 * (see usb2p_watch.c): a watch wants to sample within a few ms of a trigger
 * change so the captured context is still coherent.  The Cortex-M DWT cycle
 * counter is a free-running 32-bit counter at CPU_FREQUENCY with no setup cost
 * and no contention (unlike the RIT/TIM2 one-shots that delay_us() borrows).
 *
 * The watch scheduler compares deadlines with time_after()/time_before(), whose
 * signed-difference trick requires a counter that wraps at a power of 2.  The
 * raw cycle counter (usb2p_hires_cycles) satisfies that — it wraps at 2^32 every
 * ~44.7 s at 96 MHz and the comparisons stay correct across the wrap.
 *
 * CAVEAT: DWT CYCCNT only counts with the debug block powered; on the LPC it can
 * be frozen at 0 standalone (no debugger).  usb2p_hires_init self-tests it and,
 * when frozen, usb2p_hires_cycles falls back to a SysTick-derived cycle count
 * (same 2^32 wrap property).  Without this, watch scheduling froze after the
 * INITIAL event — see usb2p_hires_cycles. */
static uint8_t hires_inited;
/* 1 = DWT CYCCNT verified counting; 0 = use the SysTick-derived fallback.  On the
   LPC176x (Cortex-M3) the DWT cycle counter only advances when the debug block is
   powered (typically by an attached debugger); standalone it can stay frozen at 0.
   A frozen clock froze watch scheduling: the first poll runs (deadline starts 0)
   so the INITIAL event ships, but every later poll is gated on time_before(now,
   next_poll_cyc) with now==0 forever, so no change events ever fire.  We verify
   CYCCNT actually advances and fall back to a SysTick-based cycle count if not. */
static uint8_t hires_use_dwt;

static void usb2p_hires_init(void) {
  uint32_t t0, t1;
  volatile uint32_t spin;

  if (hires_inited) return;

  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  /* Self-test: burn a few hundred cycles and check CYCCNT moved.  If it didn't,
     the debug block isn't clocking the counter (no debugger attached on this
     LPC) and we must not use it — see hires_use_dwt. */
  t0 = DWT->CYCCNT;
  for (spin = 0; spin < 100; spin++) { /* deliberate busy work */ }
  t1 = DWT->CYCCNT;
  hires_use_dwt = (t1 != t0);

  hires_inited = 1;
}

/* Raw free-running cycle counter.  Wraps cleanly at 2^32, so time_after()/
   time_before() work across the wrap (every ~44.7 s at 96 MHz).  This is the
   correct clock for scheduling deadlines; convert ms/us deadlines to cycles via
   USB2P_CYCLES_PER_US / USB2P_CYCLES_PER_MS.

   DWT CYCCNT is preferred (single-instruction read, full CPU-clock resolution),
   but on the LPC it only runs with the debug block powered, so when the init
   self-test found it frozen we synthesize an equivalent counter from SysTick:
   completed-tick cycles + the intra-tick down-count.  Both terms are real elapsed
   cycles, so the sum is a genuine cycle count that wraps at 2^32 — the property
   time_after()/time_before() need.  Granularity is then bounded by how often the
   value is sampled (main-loop rate), not by the 10 ms tick: the SysTick->VAL term
   gives sub-tick resolution. */
uint32_t usb2p_hires_cycles(void) {
  uint32_t reload, val, t0, t1;

  if (hires_use_dwt) {
    return DWT->CYCCNT;
  }

  /* Read tick + intra-tick count coherently: re-read the tick and retry if the
     SysTick wrapped between the two reads (handler may have bumped `ticks`). */
  reload = SysTick->LOAD;
  do {
    t0  = getticks();
    val = SysTick->VAL;
    t1  = getticks();
  } while (t0 != t1);

  return t0 * (reload + 1u) + (reload - val);
}

int USB2P_Poll(void) {
  int cmd;

  usb2p_hires_init();

  /* usb2p_poll() runs with the USB IRQ ENABLED so its SPI reads overlap with
     USB packet shipping: while the main loop reads the next DAT chunk over
     SPI, IN-complete IRQs keep draining tx_buf to the host.  The shared
     tx_buf state is guarded by the short critical section inside
     usb2p_queue_frame (usb2p_tx_lock), not by masking the whole poll. */
  usb2p_poll();

  /* CDC2P_KickTx flushes the framed byte stream out the CDC IN endpoint when
     the transport has been upgraded; a no-op in plain serial mode. */
  CDC2P_KickTx();

  cmd = usb2p_take_command();
  return cmd;
}


/*
 *  USB Endpoint 5 Event Callback
 *   Called automatically on USB Endpoint 5 Event
 *    Parameter:       event
 */

void USB_EndPoint5 (uint32_t event) {
}


/*
 *  USB Endpoint 6 Event Callback
 *   Called automatically on USB Endpoint 6 Event
 *    Parameter:       event
 */

void USB_EndPoint6 (uint32_t event) {
}


/*
 *  USB Endpoint 7 Event Callback
 *   Called automatically on USB Endpoint 7 Event
 *    Parameter:       event
 */

void USB_EndPoint7 (uint32_t event) {
}


/*
 *  USB Endpoint 8 Event Callback
 *   Called automatically on USB Endpoint 8 Event
 *    Parameter:       event
 */

void USB_EndPoint8 (uint32_t event) {
}


/*
 *  USB Endpoint 9 Event Callback
 *   Called automatically on USB Endpoint 9 Event
 *    Parameter:       event
 */

void USB_EndPoint9 (uint32_t event) {
}


/*
 *  USB Endpoint 10 Event Callback
 *   Called automatically on USB Endpoint 10 Event
 *    Parameter:       event
 */

void USB_EndPoint10 (uint32_t event) {
}


/*
 *  USB Endpoint 11 Event Callback
 *   Called automatically on USB Endpoint 11 Event
 *    Parameter:       event
 */

void USB_EndPoint11 (uint32_t event) {
}


/*
 *  USB Endpoint 12 Event Callback
 *   Called automatically on USB Endpoint 12 Event
 *    Parameter:       event
 */

void USB_EndPoint12 (uint32_t event) {
}


/*
 *  USB Endpoint 13 Event Callback
 *   Called automatically on USB Endpoint 13 Event
 *    Parameter:       event
 */

void USB_EndPoint13 (uint32_t event) {
}


/*
 *  USB Endpoint 14 Event Callback
 *   Called automatically on USB Endpoint 14 Event
 *    Parameter:       event
 */

void USB_EndPoint14 (uint32_t event) {
}


/*
 *  USB Endpoint 15 Event Callback
 *   Called automatically on USB Endpoint 15 Event
 *    Parameter:       event
 */

void USB_EndPoint15 (uint32_t event) {
}
