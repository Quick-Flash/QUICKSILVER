#include "drv_serial.h"

#include <stm32f4xx_ll_bus.h>
#include <stm32f4xx_ll_usart.h>

#include "profile.h"
#include "project.h"
#include "usb_configurator.h"

uart_port_names_t serial_rx_port = UART_PORT_INVALID;
uart_port_names_t serial_smart_audio_port = UART_PORT_INVALID;

//FUNCTION TO SET APB CLOCK TO USART BASED ON GIVEN UART
void serial_enable_rcc(const uart_port_def_t *port) {
  switch (port->index) {
#ifdef USART1
  case 1:
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);
    break;
#endif
#ifdef USART2
  case 2:
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);
    break;
#endif
#ifdef USART3
  case 3:
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART3);
    break;
#endif
#ifdef UART4
  case 4:
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_UART4);
    break;
#endif
#ifdef UART5
  case 5:
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_UART5);
    break;
#endif
#ifdef USART6
  case 6:
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART6);
    break;
#endif
  }
}

void serial_enable_isr(const uart_port_def_t *port) {
  NVIC_SetPriorityGrouping(2);
  NVIC_SetPriority(port->irq, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
  NVIC_EnableIRQ(port->irq);
}

void handle_usart_isr(uint32_t port_index) {
  if (port_index <= UART_PORT_INVALID || port_index >= UART_PORT_MAX) {
    return;
  }

  const uart_port_def_t *port = &uart_ports[port_index - 1];

#ifdef SERIAL_RX
  extern void RX_USART_ISR(void);
  extern void TX_USART_ISR(void);

  if (serial_rx_port == port_index) {
    if (LL_USART_IsEnabledIT_TC(port->channel) && LL_USART_IsActiveFlag_TC(port->channel)) {
      LL_USART_ClearFlag_TC(port->channel);
      TX_USART_ISR();
    } else {
      RX_USART_ISR();
    }
    return;
  }
#endif

#if defined(ENABLE_SMART_AUDIO) || defined(ENABLE_TRAMP)
  extern void vtx_uart_isr(void);

  if (serial_smart_audio_port == port_index) {
    vtx_uart_isr();
    return;
  }
#endif
}

#if defined(USART1)
void USART1_IRQHandler() {
  handle_usart_isr(1);
}
#endif

#if defined(USART2)
void USART2_IRQHandler() {
  handle_usart_isr(2);
}
#endif

#if defined(USART3)
void USART3_IRQHandler() {
  handle_usart_isr(3);
}
#endif

#if defined(UART4)
void UART4_IRQHandler() {
  handle_usart_isr(4);
}
#endif

#if defined(UART5)
void UART5_IRQHandler() {
  handle_usart_isr(5);
}
#endif

#if defined(USART6)
void USART6_IRQHandler() {
  handle_usart_isr(6);
}
#endif