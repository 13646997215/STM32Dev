/* Hand-maintained board adapter. Vendor HAL/CMSIS sources remain unchanged. */
#include "board.h"
#include "main.h"
#include <math.h>
#include <string.h>
#define RX_SIZE 1024U
#define TX_SIZE 512U
typedef struct {
    USART_TypeDef *uart;DMA_Stream_TypeDef *dma;
    uint8_t rx[RX_SIZE],tx[TX_SIZE];
    volatile uint32_t wraps,tx_head,tx_tail;
    uint32_t consumed;
} SerialPort;
static SerialPort ports[2];
static FgCapture capture[2];
static volatile uint32_t errors;
static uint8_t ready;
static uint32_t lock(void){uint32_t p=__get_PRIMASK();__disable_irq();return p;}
static void unlock(uint32_t p){__set_PRIMASK(p);}
static void dma_event(unsigned port)
{
    uint32_t flags=port?DMA1->HISR:DMA2->LISR;
    uint32_t tc=port?DMA_HISR_TCIF5:DMA_LISR_TCIF2;
    uint32_t bad=port?(DMA_HISR_TEIF5|DMA_HISR_DMEIF5|DMA_HISR_FEIF5):(DMA_LISR_TEIF2|DMA_LISR_DMEIF2|DMA_LISR_FEIF2);
    uint32_t all=port?(DMA_HIFCR_CTCIF5|DMA_HIFCR_CHTIF5|DMA_HIFCR_CTEIF5|DMA_HIFCR_CDMEIF5|DMA_HIFCR_CFEIF5):
                         (DMA_LIFCR_CTCIF2|DMA_LIFCR_CHTIF2|DMA_LIFCR_CTEIF2|DMA_LIFCR_CDMEIF2|DMA_LIFCR_CFEIF2);
    if(flags&tc)ports[port].wraps++;
    if(flags&bad)errors++;
    /* Clear only observed flags: a just-arrived TC must not be erased. */
    if(port)DMA1->HIFCR=all&flags;else DMA2->LIFCR=all&flags;
}
void Board_Init(void)
{
    GPIO_InitTypeDef g={0};unsigned i;
    __HAL_RCC_GPIOA_CLK_ENABLE();__HAL_RCC_GPIOB_CLK_ENABLE();__HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_TIM2_CLK_ENABLE();__HAL_RCC_TIM4_CLK_ENABLE();__HAL_RCC_USART1_CLK_ENABLE();__HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();__HAL_RCC_DMA2_CLK_ENABLE();__HAL_RCC_SYSCFG_CLK_ENABLE();
    /* Drive speed inputs LOW before changing their function to TIM4. */
    GPIOB->BSRR=(GPIO_PIN_6|GPIO_PIN_7)<<16;
    g.Pin=GPIO_PIN_6|GPIO_PIN_7;g.Mode=GPIO_MODE_OUTPUT_PP;g.Pull=GPIO_PULLDOWN;g.Speed=GPIO_SPEED_FREQ_HIGH;HAL_GPIO_Init(GPIOB,&g);
    g.Pin=GPIO_PIN_4|GPIO_PIN_5;g.Mode=GPIO_MODE_OUTPUT_OD;g.Pull=GPIO_NOPULL;HAL_GPIO_Init(GPIOB,&g);
    GPIOB->BSRR=(GPIO_PIN_4<<16)|GPIO_PIN_5;
    TIM4->CR1=0;TIM4->PSC=39;TIM4->ARR=999;TIM4->CCR1=TIM4->CCR2=0;
    TIM4->CCMR1=(6U<<TIM_CCMR1_OC1M_Pos)|TIM_CCMR1_OC1PE|(6U<<TIM_CCMR1_OC2M_Pos)|TIM_CCMR1_OC2PE;
    TIM4->CCER=TIM_CCER_CC1E|TIM_CCER_CC2E;TIM4->EGR=TIM_EGR_UG;TIM4->CR1=TIM_CR1_ARPE|TIM_CR1_CEN;
    g.Pin=GPIO_PIN_6|GPIO_PIN_7;g.Mode=GPIO_MODE_AF_PP;g.Alternate=GPIO_AF2_TIM4;g.Pull=GPIO_PULLDOWN;HAL_GPIO_Init(GPIOB,&g);
    TIM2->PSC=79;TIM2->ARR=0xFFFFFFFFU;TIM2->EGR=TIM_EGR_UG;TIM2->CR1=TIM_CR1_CEN;
    memset(capture,0,sizeof(capture));
    g.Pin=GPIO_PIN_8|GPIO_PIN_9;g.Mode=GPIO_MODE_IT_RISING;g.Pull=GPIO_PULLUP;HAL_GPIO_Init(GPIOB,&g);
    HAL_NVIC_SetPriority(EXTI9_5_IRQn,6,0);HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
    g.Mode=GPIO_MODE_AF_PP;g.Pull=GPIO_PULLUP;g.Alternate=GPIO_AF7_USART1;g.Pin=GPIO_PIN_9|GPIO_PIN_10;HAL_GPIO_Init(GPIOA,&g);
    g.Alternate=GPIO_AF7_USART2;g.Pin=GPIO_PIN_2|GPIO_PIN_3;HAL_GPIO_Init(GPIOA,&g);
    memset(ports,0,sizeof(ports));ports[0].uart=USART1;ports[0].dma=DMA2_Stream2;ports[1].uart=USART2;ports[1].dma=DMA1_Stream5;
    DMA2->LIFCR=0x003D0000U;DMA1->HIFCR=0x00000F40U;
    for(i=0;i<2;i++){
        SerialPort *s=&ports[i];s->dma->CR=0;
        s->dma->PAR=(uint32_t)&s->uart->DR;s->dma->M0AR=(uint32_t)s->rx;s->dma->NDTR=RX_SIZE;s->dma->FCR=0;
        s->dma->CR=(4U<<DMA_SxCR_CHSEL_Pos)|DMA_SxCR_MINC|DMA_SxCR_CIRC|DMA_SxCR_TCIE|DMA_SxCR_TEIE|DMA_SxCR_DMEIE|DMA_SxCR_EN;
        s->uart->CR1=0;s->uart->CR2=0;s->uart->CR3=USART_CR3_DMAR|USART_CR3_EIE;
        s->uart->BRR=((i?HAL_RCC_GetPCLK1Freq():HAL_RCC_GetPCLK2Freq())+57600U)/115200U;
        s->uart->CR1=USART_CR1_UE|USART_CR1_TE|USART_CR1_RE|USART_CR1_PEIE;
    }
    HAL_NVIC_SetPriority(DMA2_Stream2_IRQn,7,0);HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
    HAL_NVIC_SetPriority(DMA1_Stream5_IRQn,7,0);HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
    HAL_NVIC_SetPriority(USART1_IRQn,7,0);HAL_NVIC_EnableIRQ(USART1_IRQn);
    HAL_NVIC_SetPriority(USART2_IRQn,7,0);HAL_NVIC_EnableIRQ(USART2_IRQn);
    ready=1;
}
uint32_t Board_Micros(void){return TIM2->CNT;}
void Board_Snapshot(FgCapture fg[2]){uint32_t p=lock();fg[0]=capture[0];fg[1]=capture[1];unlock(p);}
int Board_Read(unsigned port,uint8_t *b)
{
    SerialPort *s;uint32_t p,produced,available;if(port>1)return 0;s=&ports[port];p=lock();
    /* Account for a TC pending while IRQs were masked. The IRQ then sees cleared flags. */
    dma_event(port);produced=s->dma->NDTR;
    if((port?DMA1->HISR:DMA2->LISR)&(port?DMA_HISR_TCIF5:DMA_LISR_TCIF2)){
        dma_event(port);produced=s->dma->NDTR;
    }
    produced=s->wraps*RX_SIZE+RX_SIZE-produced;unlock(p);
    available=produced-s->consumed;if(!available)return 0;
    if(available>RX_SIZE){errors++;s->consumed=produced;return 0;}
    __DMB();*b=s->rx[s->consumed&(RX_SIZE-1U)];s->consumed++;return 1;
}
int Board_Write(unsigned port,const uint8_t *data,size_t n)
{
    SerialPort *s;uint32_t p;size_t i;if(port>1||n>TX_SIZE)return 0;s=&ports[port];p=lock();
    if(n>TX_SIZE-(uint32_t)(s->tx_head-s->tx_tail)){unlock(p);return 0;}
    for(i=0;i<n;i++)s->tx[(s->tx_head+i)&(TX_SIZE-1U)]=data[i];
    __DMB();s->tx_head+=(uint32_t)n;s->uart->CR1|=USART_CR1_TXEIE;unlock(p);return 1;
}
static void uart_irq(unsigned port)
{
    SerialPort *s=&ports[port];uint32_t sr=s->uart->SR;
    if(sr&(USART_SR_ORE|USART_SR_NE|USART_SR_FE|USART_SR_PE)){volatile uint32_t discard=s->uart->DR;(void)discard;errors++;}
    if((sr&USART_SR_TXE)&&(s->uart->CR1&USART_CR1_TXEIE)){
        if(s->tx_tail!=s->tx_head){s->uart->DR=s->tx[s->tx_tail&(TX_SIZE-1U)];s->tx_tail++;}
        else s->uart->CR1&=~USART_CR1_TXEIE;
    }
}
void USART1_IRQHandler(void){uart_irq(0);}void USART2_IRQHandler(void){uart_irq(1);}
void DMA2_Stream2_IRQHandler(void){dma_event(0);}void DMA1_Stream5_IRQHandler(void){dma_event(1);}
void HAL_GPIO_EXTI_Callback(uint16_t pin)
{if(pin==GPIO_PIN_8)Fg_Edge(&capture[0],Board_Micros());else if(pin==GPIO_PIN_9)Fg_Edge(&capture[1],Board_Micros());}
void Board_Stop(void){if(ready){TIM4->CCR1=TIM4->CCR2=0;TIM4->EGR=TIM_EGR_UG;}}
void Board_Apply(const float duty[2],const int8_t direction[2],const RobotConfig *c)
{
    unsigned i;uint16_t pin;int release;
    if(!ready)return;
    for(i=0;i<2;i++){
        pin=i?GPIO_PIN_5:GPIO_PIN_4;release=direction[i]>0?c->forward_release[i]:!c->forward_release[i];
        HAL_GPIO_WritePin(GPIOB,pin,release?GPIO_PIN_SET:GPIO_PIN_RESET);
    }
    TIM4->CCR1=Course_Finite(duty[0])?(uint32_t)(Course_Clamp(duty[0],0,c->max_duty)*1000):0;
    TIM4->CCR2=Course_Finite(duty[1])?(uint32_t)(Course_Clamp(duty[1],0,c->max_duty)*1000):0;
}
void Board_Led(int on){HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,on?GPIO_PIN_RESET:GPIO_PIN_SET);}
int Board_Key(void){return HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0)==GPIO_PIN_RESET;}
uint32_t Board_Errors(void){return errors;}
void Board_Fatal(void){Board_Stop();__disable_irq();for(;;){__NOP();}}
