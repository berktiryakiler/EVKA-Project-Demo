
void setup() {

  Serial.begin(9600);
  while (!Serial) {
    ;
  }

}


int a[5] = {1323,3435,2231,-14, 55};

const unsigned int BUFFER_SIZE = 10; // payload size
byte data[BUFFER_SIZE];

void loop() {
  /*a[0] = random(140);
  a[1] = random(2350);
  a[2] = random(1200);
  a[3] = random(3232);
  a[4] = random(-100,-25);*/
  
  delay(500);
  a[0] +=1;
  a[1] +=1;
  a[2] +=1;
  a[3] +=1;
  a[4] +=1;
  memcpy(&data[0], &a[0], 10);
  
  Serial.write(data, 10);
  
}
