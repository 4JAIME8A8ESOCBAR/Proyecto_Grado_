#include <DHT.h>           //Cargamos la librería DHT
#define DHTTYPE   DHT22   //Definimos el modelo DHT22
#define DHTPIN    19    // Se define el pin D4 del ESP32 para conectar el sensor DHT22
DHT dht(DHTPIN, DHTTYPE, 22); 
void setup()
{
  Serial.begin(115200);   //Se inicia la comunicación serial 
  dht.begin(); 
}
void loop()
{
 float h = dht.readHumidity(); //Se lee la humedad y se asigna el valor a "h"
 float t = dht.readTemperature(); //Se lee la temperatura y se asigna el valor a "t"
 //Se imprimen las variables
  Serial.println("Humedad: "); 
  Serial.println(h);
  Serial.println("Temperatura: ");
  Serial.println(t);
  delay(2000);  
}