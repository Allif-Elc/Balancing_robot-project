unsigned char Sensor;
int sensor[8];
int threshold[8] = {};
int data_putih[8] = {659, 663, 470, 524, 671, 616, 576, 661};
int data_hitam[8] = {989, 990, 976, 976, 987, 964, 925, 940};

void hitung_treshold()
{
  for (int i = 0; i < 8; i++)
  {
    threshold[i] = data_hitam[i] - ((data_hitam[i] - data_putih[i]) * 0.8); //0.1 - 0.9 makin besar makin sensitif
  }
}

void baca_sensor()
{
  sensor[0] = analogRead(A7);
  sensor[1] = analogRead(A5);
  sensor[2] = analogRead(A6);
  sensor[3] = analogRead(A3);
  sensor[4] = analogRead(A4);
  sensor[5] = analogRead(A1);
  sensor[6] = analogRead(A2);
  sensor[7] = analogRead(A0);

//   for (int i = 0; i < 8; i++)
//    {
 //     Serial.print(sensor[i]);
//      Serial.print("  | ");
//    }
//    Serial.println("");
}

void baca_garis()
{
  baca_sensor();
  for (int i = 0; i < 8; i++) {
    if (sensor[i] > threshold[i]) sensor[i] = 1;
    else sensor[i] = 0;
    //Serial.print(sensor[i]);
  }
  //Serial.println("");
}


void jalanKiri()
{
  int error;
  static int lastErr;
  baca_garis();
  Sensor = (sensor[7] * 1) + (sensor[6] * 2) + (sensor[5] * 4) + (sensor[4] * 8) + (sensor[3] * 16) + (sensor[2] * 32) + (sensor[1] * 64) + (sensor[0] * 128);
  Sensor = ~Sensor;
  switch (Sensor)
  {
    case 0b11111110: error = 6;   break;
    case 0b11111100: error = 5;   break;
    case 0b11111101: error = 4;   break;
    case 0b11111001: error = 3;   break;
    case 0b11111011: error = 2;   break;
    case 0b11110011: error = 2;   break;
    case 0b11110111: error = 1;   break;
    case 0b11100111: error = 0;   break;
    case 0b11101111: error = 1;   break;
    case 0b11001111: error = -2;  break;
    case 0b11011111: error = -2;  break;
    case 0b10011111: error = -3;  break;
    case 0b10111111: error = -4;  break;
    case 0b00111111: error = -5;  break;
    case 0b01111111: error = -6;  break;
    case 0b01111110: error = 99;  break;
    case 0b00000000: error = 99;  break;
    case 0b00111100: error = 99;  break;
    case 0b00011000: error = 99;  break;
    default : error = lastErr; break;
  }
  lastErr = error;

  if (error == 99)
  {
      g_carstate = enSTOP;
  }
  else
  {
    if (error > 1 && error < 5 )
    {
      g_carstate = enLEFT;
    }
    else if (error >= 5 && error < 7 )
    {
      g_carstate = enLEFT;
    }
    else if (error <= 1 && error >= -1)
    {
      g_carstate = enBACK;
    }
    else  if (error >= -5 && error < -1 )
    {
      g_carstate = enRIGHT;
    }
    else  if (error > -7 && error < -5 )
    {
      g_carstate = enRIGHT;
    }
    else
    {
      g_carstate = enSTOP;
    }
  }
}


void jalanKanan()
{
  int error;
  static int lastErr;
  baca_garis();
  Sensor = (sensor[7] * 1) + (sensor[6] * 2) + (sensor[5] * 4) + (sensor[4] * 8) + (sensor[3] * 16) + (sensor[2] * 32) + (sensor[1] * 64) + (sensor[0] * 128);
  Sensor = ~Sensor;
  switch (Sensor)
  {
    case 0b11111110: error = 5;   break;
    case 0b11111100: error = 4;   break;
    case 0b11111101: error = 3;   break;
    case 0b11111001: error = 3;   break;
    case 0b11111011: error = 3;   break;
    case 0b11110011: error = 3;   break;
    case 0b11110111: error = 3;   break;
    case 0b11100111: error = 2;   break;
    case 0b11101111: error = 1;   break;
    case 0b11001111: error = 0;  break;
    case 0b11011111: error = -1;  break;
    case 0b10011111: error = -2;  break;
    case 0b10111111: error = -2;  break;
    case 0b00111111: error = -3;  break;
    case 0b01111111: error = -4;  break;
    case 0b01111110: error = 99;  break;
    case 0b00000000: error = 99;  break;
    case 0b01111100: error = 99;  break;
//    case 0b01111100: error = 99;  break;
    default : error = lastErr; break;
  }
  lastErr = error;

  if (error == 99)
  {
      g_carstate = enSTOP;
  }
  else
  {
    if (error > 1 && error < 5 )
    {
      g_carstate = enLEFT;
    }
    else if (error >= 5 && error < 7 )
    {
      g_carstate = enLEFT;
    }
    else if (error <= 1 && error >= -1)
    {
      g_carstate = enBACK;
    }
    else  if (error >= -5 && error < -1 )
    {
      g_carstate = enRIGHT;
    }
    else  if (error > -7 && error < -5 )
    {
      g_carstate = enRIGHT;
    }
    else
    {
      g_carstate = enSTOP;
    }
  }
}


