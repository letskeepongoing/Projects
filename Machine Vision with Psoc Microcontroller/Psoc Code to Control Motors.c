
#include "project.h"

int theta1(float angle1){
    int compare;
    int min_comp = 1500;
    int max_comp = 7000; //could be 7050
    int maxA = 180;
    int minA = 0;
    compare = ((max_comp-min_comp)/(maxA-minA))*(angle1-minA)+min_comp;
    return compare;
}

int theta2(float angle1){
    int compare;
    int min_comp = 900;
    int max_comp = 6800; //could be 7050
    int maxA = -90;
    int minA = 90;
    compare = ((max_comp-min_comp)/(maxA-minA))*(angle1-minA)+min_comp;
    return compare;
}

int main(void)
{
    int count;
    //int count2;
    int count_dif;
    int target_1= 500; //positions for the gear to go too
    int target_2 = 20300;
    int Error;
    float cpr = 10300.0;
    float turn = cpr/4;
    float Kp=1.0;
    /////////////////////////
    uint8 Receive;
    ///////////////////////////
    int time;
    int Speed;
    int hold;
    int hold2;
    int ff = 2;
    QuadDec_1_Start();
    LCD_Char_1_Start();
    PWM_1_Start();
    PWM_2_Start();
    UART_1_Start();

    int xx=1;
    for(;;)
    {
        while(xx)
        {
            while(ff > 0)
            {
                Receive = UART_1_GetChar();
                while (Receive == 0)
                {
                    Receive = UART_1_GetChar();
                }
                LCD_Char_1_ClearDisplay();
                LCD_Char_1_Position(0,0);
                hold = Receive;
                LCD_Char_1_PrintNumber(Receive);
            
                CyDelay(1000);
                Receive = UART_1_GetChar();
                hold2 = Receive;
                LCD_Char_1_Position(1,0);
                LCD_Char_1_PrintNumber(Receive);
                CyDelay(1000);
                ff=ff-1;
            }
      
            xx = 0;
            LCD_Char_1_ClearDisplay();
            LCD_Char_1_Position(0,0);
            LCD_Char_1_PrintNumber(200);
            CyDelay(1000);
            
        }
        
        LCD_Char_1_ClearDisplay();
        LCD_Char_1_Position(0,0);
        LCD_Char_1_PrintNumber(hold);
        LCD_Char_1_Position(1,0);
        LCD_Char_1_PrintNumber(hold2);
        
        PWM_2_WriteCompare1(theta1((float)hold)); //1500 most clockwise position // most counter clockwise 6900
        CyDelay(1500);       
        PWM_2_WriteCompare2(theta2((float)-hold2)); //1500 most clockwise position // most counter clockwise 6900
        CyDelay(1500);
        
        time =0;
        QuadDec_1_SetCounter(0);
        while (time < 2000){
            count = QuadDec_1_GetCounter();

            if (count < target_1 + 300 ){
                PWM_1_WriteCompare1(60); //
                PWM_1_WriteCompare2(0);
                
                
            }
            else if(count > target_1 - 300){
                PWM_1_WriteCompare1(0);
                PWM_1_WriteCompare2(60);
                
            }
            else {
                PWM_1_WriteCompare1(0);
                PWM_1_WriteCompare2(0);
            }
            
            CyDelay(10);
            time = time + 10;
            LCD_Char_1_ClearDisplay();
            LCD_Char_1_Position(0,0);
            LCD_Char_1_PrintNumber(count);
        }
        
        time =0;
        while (time < 10000){
            count = QuadDec_1_GetCounter();

            if (count < target_2 + 300 ){
                PWM_1_WriteCompare1(60);
                PWM_1_WriteCompare2(0);     
                
                
            }  
            else if(count > target_2 - 300){
                PWM_1_WriteCompare1(0);
                PWM_1_WriteCompare2(60);
                
            }
            else {
                PWM_1_WriteCompare1(0);
                PWM_1_WriteCompare2(0);
            }
            
            CyDelay(10);
            time = time + 10;
            LCD_Char_1_ClearDisplay();
            LCD_Char_1_Position(0,0);
            LCD_Char_1_PrintNumber(count);
        }
        
        time =0;
        while (time < 10000){
            count = QuadDec_1_GetCounter();

            if (count < target_1 + 300 ){
                PWM_1_WriteCompare1(60); //
                PWM_1_WriteCompare2(0);
                
                
            }
            else if(count > target_1 - 300){
                PWM_1_WriteCompare1(0);
                PWM_1_WriteCompare2(60);
                
            }
            else {
                PWM_1_WriteCompare1(0);
                PWM_1_WriteCompare2(0);
            }
            
            CyDelay(10);
            time = time + 10;
            LCD_Char_1_ClearDisplay();
            LCD_Char_1_Position(0,0);
            LCD_Char_1_PrintNumber(count);
        }
        
        PWM_1_WriteCompare1(0);
        PWM_1_WriteCompare2(0);
        
        PWM_2_WriteCompare1(theta1(0.0)); 
        CyDelay(2000);
        PWM_2_WriteCompare2(theta2(0.0)); 
        CyDelay(2000);
        
        break;
        
    }
}

/* [] END OF FILE */
