/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/
#include "project.h"
#include <stdlib.h>
#include <stdbool.h>

// NMEA definitions
#define NMEA_MAX_SIZE             82
#define NMEA_START_DELIMITER      '$'
#define NMEA_END_DELIMITER        0x0A
#define NMEA_CHECKSUM_DELIMITER   '*'
#define NMEA_FIELD_DELIMITER      ','
#define NMEA_MSG_NAME_SIZE        4

#define NMEA_SHBCC_EMPTY            "SHBCC"
#define NMEA_SHBCC_CMD              1
#define NMEA_SHBCC_VALUE            1

#define BOILER_PWR_ON   0
#define BOILER_PWR_OFF  1
#define LED_ON          0
#define LED_OFF         1

static const char cmd_on[] = "ON";
static const char cmd_off[] = "OFF";
static const char nmea_shbcc_empty[] = NMEA_SHBCC_EMPTY;
char cmd_buf[NMEA_MAX_SIZE];

char NMEA_buffer[NMEA_MAX_SIZE];
char NMEA_SHBCC[NMEA_MAX_SIZE] = NMEA_SHBCC_EMPTY;
uint8 NMEA_pointer;
bool NMEA_packet_received = false;

void NMEA_handle_packet(char *packet, char *NMEA_data);
void NMEA_GetField(char *packet, uint8 field, char *result);

bool check_cmd(const char *cmd);
void blink_red(uint16 delay);

CY_ISR(isr_rf)
{    
    if (NMEA_pointer >= NMEA_MAX_SIZE) NMEA_pointer = 0;
    NMEA_buffer[NMEA_pointer] = UART_RF_GetChar();
    NMEA_buffer[NMEA_pointer + 1] = 0;    
    switch(NMEA_buffer[NMEA_pointer])
    {
        case NMEA_START_DELIMITER:
        NMEA_pointer = 0;
        break;
        
        case NMEA_END_DELIMITER:
        NMEA_handle_packet(NMEA_buffer, NMEA_SHBCC);
        NMEA_packet_received = true; 
        break;
        
        default:
        NMEA_pointer++; 
        break;
    }
}

CY_ISR(isr_deadline)
{
    Control_Reg_Boiler_Write(BOILER_PWR_ON);
    Timer_Deadline_ClearInterrupt(Timer_Deadline_INTR_MASK_TC);
}

CY_ISR(isr_blink)
{
    Pin_LED_red_Write(LED_OFF);
    Timer_Blink_ClearInterrupt(Timer_Blink_INTR_MASK_TC);
}

int main(void)
{    
    CyGlobalIntEnable; /* Enable global interrupts. */
    UART_RF_Start();
    Timer_Deadline_Start();
    Timer_Blink_Start();
    
    isr_rf_StartEx(isr_rf);
    isr_deadline_StartEx(isr_deadline);
    isr_blink_StartEx(isr_blink);
    
    blink_red(200);
    
    for(;;)
    {
        if (NMEA_packet_received)
        {
            NMEA_packet_received = false;
            NMEA_GetField(NMEA_SHBCC, NMEA_SHBCC_CMD, cmd_buf);
            if (check_cmd(cmd_on))
            {
                Control_Reg_Boiler_Write(!Control_Reg_Boiler_Read());
            }
            if (check_cmd(cmd_off))
            {
                Control_Reg_Boiler_Write(BOILER_PWR_OFF);
                Timer_Deadline_WriteCounter(0);
            }
            NMEA_SHBCC[0] = 0;
            strlcat(NMEA_SHBCC, nmea_shbcc_empty, NMEA_MAX_SIZE);
        }
    }
}

void blink_red(uint16 delay)
{
    Pin_LED_red_Write(LED_ON);
    Timer_Blink_WritePeriod(delay);
    Timer_Blink_Start();
}
    

bool check_cmd(const char *cmd)
{
    return !strncmp(cmd_buf, cmd, sizeof(cmd) - 1);
}

void NMEA_GetField(char *packet, uint8 field, char *result)
{
    uint8 i;
    uint8 count = 0;
    
    // Search field
    for (i = 0; (i < NMEA_MAX_SIZE) & (count < field); i++)
    {
        if (packet[i] == NMEA_FIELD_DELIMITER) count++;
    }
    
    // Measure field size
    for (count = 0; count < NMEA_MAX_SIZE; count++)
    {
        if (packet[i + count] == NMEA_FIELD_DELIMITER) break;
        if (packet[i + count] == 0u) break;
    }
    strlcpy(result, packet + i, count + 1);  // Add 1 to count for null terminator
}

void NMEA_handle_packet(char *packet, char *NMEA_data)
{
    uint8 i, n;
    uint8 error = 0;
    uint8 checksum = 0;
    char *packet_checksum;
    char calculated_checksum[3];
        
    // Check if appropriate packet is handled
    if (!strncmp(packet, NMEA_data, NMEA_MSG_NAME_SIZE))
    {
        // Check for receive errors
        for(i = 0; i < NMEA_MAX_SIZE; i++)
        {
            if ((packet[i] < 32) & (packet[i] != 0x0D) & (packet[i] != NMEA_END_DELIMITER)) 
            {
                error++;
                break;
            }
            if (packet[i] != NMEA_END_DELIMITER) break;
        }
        
        // Validate checksum and cut packet if no receive errors
        if (!error)
        {
            // Find checksum field
            packet_checksum = memchr(packet, NMEA_CHECKSUM_DELIMITER, NMEA_MAX_SIZE) + 1;
            i = (uint8)(packet_checksum - packet);
            // Cut string to NMEA_CHECKSUM_DELIMITER
            packet[i-1] = 0;
            
            // Calculate checksum and compare
            for (n = 0; n < i; n++) checksum ^= packet[n];
            itoa(checksum, calculated_checksum, 16);
            if(!strcmp(calculated_checksum, packet_checksum)) error++;
        }   
        
        // Copy buffer to NMEA packet if no errors found
        if (!error) strlcpy(NMEA_data, packet, NMEA_MAX_SIZE);
        
        // Blink red if receive errors occured
        if(error) blink_red(200);
    }
}

/* [] END OF FILE */
