from ida_segment import *
from idc import *
import ida_name, ida_bytes

def create_var(address, name):
    create_dword(address)
    set_name(address, name, ida_name.SN_FORCE)

def add_segm_safe(para, start, end, name: str, sclass: str, flags: int = 0):
    segment = get_segm_by_name(name)
    if segment:
        return # this segment already exists

    add_segm(para, start, end, name, sclass, flags)

def define_gpio_instance(gpiox, num):
    create_var(gpiox + 0x0, f"GPIO{num}_CTL")
    create_var(gpiox + 0x4, f"GPIO{num}_OMODE")
    create_var(gpiox + 0x8, f"GPIO{num}_OSPD0")
    create_var(gpiox + 0xC, f"GPIO{num}_PUD")
    create_var(gpiox + 0x10, f"GPIO{num}_ISTAT")
    create_var(gpiox + 0x14, f"GPIO{num}_OCTL")
    create_var(gpiox + 0x18, f"GPIO{num}_BOP")
    create_var(gpiox + 0x1C, f"GPIO{num}_LOCK")
    create_var(gpiox + 0x20, f"GPIO{num}_AFSEL0")
    create_var(gpiox + 0x24, f"GPIO{num}_AFSEL1")
    create_var(gpiox + 0x28, f"GPIO{num}_BC")
    create_var(gpiox + 0x2C, f"GPIO{num}_TG")
    create_var(gpiox + 0x3C, f"GPIO{num}_OSPD1")
    
def define_timer_instance(timerx, num):
    create_var(timerx + 0x00000000, f"TIMER{num}_CTL0")
    create_var(timerx + 0x00000004, f"TIMER{num}_CTL1")
    create_var(timerx + 0x00000008, f"TIMER{num}_SMCFG")
    create_var(timerx + 0x0000000C, f"TIMER{num}_DMAINTEN")
    create_var(timerx + 0x00000010, f"TIMER{num}_INTF")
    create_var(timerx + 0x00000014, f"TIMER{num}_SWEVG")
    create_var(timerx + 0x00000018, f"TIMER{num}_CHCTL0")
    create_var(timerx + 0x0000001C, f"TIMER{num}_CHCTL1")
    create_var(timerx + 0x00000020, f"TIMER{num}_CHCTL2")
    create_var(timerx + 0x00000024, f"TIMER{num}_CNT")
    create_var(timerx + 0x00000028, f"TIMER{num}_PSC")
    create_var(timerx + 0x0000002C, f"TIMER{num}_CAR")
    create_var(timerx + 0x00000030, f"TIMER{num}_CREP")
    create_var(timerx + 0x00000034, f"TIMER{num}_CH0CV")
    create_var(timerx + 0x00000038, f"TIMER{num}_CH1CV")
    create_var(timerx + 0x0000003C, f"TIMER{num}_CH2CV")
    create_var(timerx + 0x00000040, f"TIMER{num}_CH3CV")
    create_var(timerx + 0x00000044, f"TIMER{num}_CCHP")
    create_var(timerx + 0x00000048, f"TIMER{num}_DMACFG")
    create_var(timerx + 0x0000004C, f"TIMER{num}_DMATB")
    create_var(timerx + 0x00000050, f"TIMER{num}_IRMP")
    create_var(timerx + 0x000000FC, f"TIMER{num}_CFG")

# -- CODE SEGMENTS --
add_segm_safe(0, 0x00000000, 0x000FFFFF, "ALIAS_SYS", "CODE")
##add_segm_safe(0, 0x00100000, 0x07FFFFFF, "Reserved", "CONST", SEG_NULL)

add_segm_safe(0, 0x08000000, 0x0801FFFF, "CODE", "CODE")

add_segm_safe(0, 0x1FFFEC00, 0x1FFFF7FF, "SYSTEM", "CODE")
add_segm_safe(0, 0x1FFFF800, 0x1FFFFBFF, "OPTIONS", "DATA")
#add_segm_safe(0, 0x1FFFFC00, 0x1FFFFFFF, "Reserved", "CONST", SEG_NULL)

# -- SRAM SEGMENTS --
add_segm_safe(0, 0x20000000, 0x20003FFF, "SRAM", "DATA")

# load SRAM data from decompressed data section in CODE
bin_file = open("hi.bin", "rb")

file_data = bin_file.read()
for idx in range(len(file_data)):
    ida_bytes.put_byte(0x20000000 + idx, file_data[idx])
    
bin_file.close()

#add_segm_safe(0, 0x20004000, 0x3FFFFFFF, "Reserved", "CONST", SEG_NULL)

# -- PERIPHERAL SEGMENTS --
# APB1 bus
#add_segm_safe(0, 0x4000CC00, 0x4000FFFF, "Reserved", "DATA") 
add_segm_safe(0, 0x4000C800, 0x4000CBFF, "CTC", "DATA") 
#add_segm_safe(0, 0x4000C400, 0x4000C7FF, "Reserved", "CONST", SEG_NULL) 
#add_segm_safe(0, 0x4000C000, 0x4000C3FF, "Reserved", "CONST", SEG_NULL) 
#add_segm_safe(0, 0x40008000, 0x4000BFFF, "Reserved", "CONST", SEG_NULL) 
#add_segm_safe(0, 0x40007C00, 0x40007FFF, "Reserved", "CONST", SEG_NULL) 
add_segm_safe(0, 0x40007800, 0x40007BFF, "CEC", "DATA") 
add_segm_safe(0, 0x40007400, 0x400077FF, "DAC0", "DATA") 
add_segm_safe(0, 0x40007000, 0x400073FF, "PMU", "DATA") 
#add_segm_safe(0, 0x40006400, 0x40006FFF, "Reserved", "CONST", SEG_NULL)
#add_segm_safe(0, 0x40006000, 0x400063FF, "Reserved", "CONST", SEG_NULL) 
#add_segm_safe(0, 0x40005C00, 0x40005FFF, "Reserved", "CONST", SEG_NULL) 
add_segm_safe(0, 0x40005800, 0x40005BFF, "I2C1", "DATA") 
add_segm_safe(0, 0x40005400, 0x400057FF, "I2C0", "DATA") 
#add_segm_safe(0, 0x40004800, 0x400053FF, "Reserved", "CONST", SEG_NULL) 
add_segm_safe(0, 0x40004400, 0x400047FF, "USART1", "DATA") 
#add_segm_safe(0, 0x40004000, 0x400043FF, "Reserved", "CONST", SEG_NULL) 
#add_segm_safe(0, 0x40003C00, 0x40003FFF, "Reserved", "CONST", SEG_NULL) 
add_segm_safe(0, 0x40003800, 0x40003BFF, "SPI1", "DATA") 
#add_segm_safe(0, 0x40003400, 0x400037FF, "Reserved", "CONST", SEG_NULL) 
add_segm_safe(0, 0x40003000, 0x400033FF, "FWDGT", "DATA") 
add_segm_safe(0, 0x40002C00, 0x40002FFF, "WWDGT", "DATA") 
add_segm_safe(0, 0x40002800, 0x40002BFF, "RTC", "DATA") 
#add_segm_safe(0, 0x40002400, 0x400027FF, "Reserved", "CONST", SEG_NULL) 
add_segm_safe(0, 0x40002000, 0x400023FF, "TIMER13", "DATA") 
define_timer_instance(0x40002000, "13")
#add_segm_safe(0, 0x40001400, 0x40001FFF, "Reserved", "CONST", SEG_NULL) 
add_segm_safe(0, 0x40001000, 0x400013FF, "TIMER5", "DATA") 
define_timer_instance(0x40001000, "5")
#add_segm_safe(0, 0x40000800, 0x40000FFF, "Reserved", "CONST", SEG_NULL) 
add_segm_safe(0, 0x40000400, 0x400007FF, "TIMER2", "DATA") 
define_timer_instance(0x40000400, "2")
add_segm_safe(0, 0x40000000, 0x400003FF, "TIMER1", "DATA")
define_timer_instance(0x40000000, "1")

# APB2 bus
#add_segm_safe(0, 0x40018000, 0x4001FFFF, "Reserved", "CONST", SEG_NULL)
#add_segm_safe(0, 0x40015C00, 0x40017FFF, "Reserved", "CONST", SEG_NULL)
#add_segm_safe(0, 0x40014C00, 0x40015BFF, "Reserved", "CONST", SEG_NULL)
add_segm_safe(0, 0x40014800, 0x40014BFF, "TIMER16", "DATA")
define_timer_instance(0x40014800, "16")
add_segm_safe(0, 0x40014400, 0x400147FF, "TIMER15", "DATA")
define_timer_instance(0x40014400, "15")
add_segm_safe(0, 0x40014000, 0x400143FF, "TIMER14", "DATA")
define_timer_instance(0x40014000, "14")
#add_segm_safe(0, 0x40013C00, 0x40013FFF, "Reserved", "CONST", SEG_NULL)
add_segm_safe(0, 0x40013800, 0x40013BFF, "USART0", "DATA")
#add_segm_safe(0, 0x40013400, 0x400137FF, "Reserved", "CONST", SEG_NULL)
add_segm_safe(0, 0x40013000, 0x400133FF, "SPI0_I2S0", "DATA")
add_segm_safe(0, 0x40012C00, 0x40012FFF, "TIMER0", "DATA")
define_timer_instance(0x40012C00, "0")
#add_segm_safe(0, 0x40012800, 0x40012BFF, "Reserved", "CONST", SEG_NULL)
add_segm_safe(0, 0x40012400, 0x400127FF, "ADC", "DATA")
#add_segm_safe(0, 0x40010800, 0x400123FF, "Reserved", "CONST", SEG_NULL)
add_segm_safe(0, 0x40010400, 0x400107FF, "EXTI", "DATA")
add_segm_safe(0, 0x40010000, 0x400103FF, "SYSCFG_CMP", "DATA")

# AHB1 bus
#add_segm_safe(0, 0x40024400, 0x47FFFFFF, "Reserved", "CONST", SEG_NULL)
add_segm_safe(0, 0x40024000, 0x400243FF, "TSI", "DATA")
#add_segm_safe(0, 0x40023400, 0x40023FFF, "Reserved", "CONST", SEG_NULL)
add_segm_safe(0, 0x40023000, 0x400233FF, "CRC", "DATA")
#add_segm_safe(0, 0x40022400, 0x40022FFF, "Reserved", "CONST", SEG_NULL)
add_segm_safe(0, 0x40022000, 0x400223FF, "FMC", "DATA")
#add_segm_safe(0, 0x40021400, 0x40021FFF, "Reserved", "CONST", SEG_NULL)
add_segm_safe(0, 0x40021000, 0x400213FF, "RCU", "DATA")
#add_segm_safe(0, 0x40020400, 0x40020FFF, "Reserved", "CONST", SEG_NULL)
add_segm_safe(0, 0x40020000, 0x400203FF, "DMA", "DATA")

# AHB2 bus
#add_segm_safe(0, 0x48001800, 0x4FFFFFFF, "Reserved", "CONST", SEG_NULL)
add_segm_safe(0, 0x48001400, 0x480017FF, "GPIOF", "DATA")
define_gpio_instance(0x48001400, "F")
#add_segm_safe(0, 0x48001000, 0x480013FF, "Reserved", "CONST", SEG_NULL)
add_segm_safe(0, 0x48000C00, 0x48000FFF, "GPIOD", "DATA")
define_gpio_instance(0x48000C00, "D")
add_segm_safe(0, 0x48000800, 0x48000BFF, "GPIOC", "DATA")
define_gpio_instance(0x48000800, "C")
add_segm_safe(0, 0x48000400, 0x480007FF, "GPIOB", "DATA")
define_gpio_instance(0x48000400, "B")
add_segm_safe(0, 0x48000000, 0x480003FF, "GPIOA", "DATA")
define_gpio_instance(0x48000000, "A")

# AHB1 bus
#add_segm_safe(0, 0x50040000, 0x5FFFFFFF, "Reserved", "CONST", SEG_NULL)
add_segm_safe(0, 0x50000000, 0x5003FFFF, "USBFS", "DATA")

# -- EXTERNAL RAM SEGMENTS --
#add_segm_safe(0, 0x60000000, 0x9FFFFFFF, "Reserved", "CONST", SEG_NULL)

# -- EXTERNAL DEVICE SEGMENTS --
#add_segm_safe(0, 0xA0000000, 0xDFFFFFFF, "Reserved", "CONST", SEG_NULL)

# -- INTERNAL PERIPHERAL SEGMENTS --
add_segm_safe(0, 0xE0000000, 0xE00FFFFF, "CORTEX", "DATA")