/* Test at DS18B20 completed
	Presence = Start();
	HAL_us_delay(1);
	Write(0xCC); //skip ROM
	Write(0x44);
	HAL_us_delay(800);
			
	Presence = Start();
	HAL_us_delay(1);
	Write(0xCC);
	Write(0xBE);
	
	Temp_byte1 = Read();
	Temp_byte2 = Read();
	TEMP = (Temp_byte2<<8)|Temp_byte1;
	temperature = (float)TEMP/16;
	
		
	HAL_us_delay(3000);   */
	