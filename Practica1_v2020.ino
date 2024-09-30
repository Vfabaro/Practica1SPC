#include <AFMotor.h>//solamente si utiliza el shield arduino motor v1. Si se usa algún chip puente H (por ejemplo L293D) se recomienda utilizar la librería TimerThree.zip
#include <TimerOne.h>
#include <PID_v1.h>
#include <SimpleKalmanFilter.h>

#define SALIDA_MAX 3                 //Salida máxima del PID (luego se le resta la Zona Muerta del Motor si hace falta)
#define TENSION_MAX 3.5                //Tensión máxima de salida del pwm
#define ZONA_MUERTA 1.2                 //ZONA MUERTA DEL MOTOR  4.25
#define TOLERANCIA 2.0
#define REF_MAX 90.0
#define REF_MIN -90.0
#define SENSOR_MAX 850.0
#define SENSOR_MIN 145.0
#define SENSOR_MEDIO ((SENSOR_MAX-SENSOR_MIN)/2.0+SENSOR_MIN)
#define POT2GRAD ((REF_MAX-REF_MIN)/(SENSOR_MAX-SENSOR_MIN)) //escalado de potenciómetro a grados

#define Ts 2  //Periodo de muestreo de 2 ms


//Define Variables we'll be connecting to PID
double setpoint=0.0, entrada=0.0, salida=0.0;

//Parametros PID

float Kp=0.5 , Ki=0.03, Kd=0.01;


//Variables leídas y error entre ellas
float ref=0.0, sensor=0.0, error=0.0, aux=0.0, sensor1=0.0, sensor2=0.0,sensor3=0.0;
int pot=0;

byte pwm_motor;
byte contador_tx=0;
byte contador_rx=0;
bool bandera_control=0;//activa tarea lazo de control
bool bandera_tx=0;// activa tarea transmisión de datos
bool bandera_rx=0;// activa tarea recepción de datos
char buffer_tx[20];
char buffer_rx[20];

AF_DCMotor motor(3, MOTOR34_8KHZ);//define la frecuencia de la señal PWM

const byte numChars = 20;
char receivedChars[20];   // an array to store the received data
float dataNumber = 0.0;

//Specify the links and initial tuning parameters
PID myPID(&entrada, &salida, &setpoint, Kp, Ki, Kd, DIRECT);

/*SimpleKalmanFilter(e_mea, e_est, q);
 e_mea: Measurement Uncertainty 
 e_est: Estimation Uncertainty 
 q: Process Noise */
SimpleKalmanFilter simpleKalmanFilter(2, 2, 0.01);
float SensorKalman = 0.0;

void setup() 
{
  motor.setSpeed(0);
  motor.run(RELEASE);

  //Inicializa comunicación serie a 115200 bits por segundo
  Serial.begin(115200);
  Serial.setTimeout(10);
  
  myPID.SetSampleTime(Ts); //Seteo período de muestreo del PID a 1 ms
  myPID.SetOutputLimits(-(SALIDA_MAX-ZONA_MUERTA), (SALIDA_MAX-ZONA_MUERTA));  //Seteo los límites de salida del PID (teniendo en cuenta la zona muerta del Motor)

  //turn the PID on
  myPID.SetMode(AUTOMATIC);
 
  Timer1.initialize(2000); // configura un timer de 1 ms
  Timer1.attachInterrupt( timerIsr ); // asocia una la interrupción de un timer a una rutina de servicio de interrupción
}
 
void loop()
{
  if(bandera_control==1) // Lazo de control
  {
    //lee referencia de HyperIMU ([-90,90] en este caso, actualizada en "dataNumber" cada 100ms)
    ref = dataNumber;
    //ref = ref_U.number;
    //accion_U.number=0.0;
    
    //Para probar el control sin Matlab, se puede obtener la referencia usando otro potenciómetro conectado a un pin del conversor A/D
    //ref=(float)analogRead(A0);
    //ref = ref*(270.0 / 1023.0) - 135.0;  //conversión de ref [0,1023] a grados (-135° a 135° en este caso particular)
    
    //opcional, generalmente se utilizan fines de carrera
    if(ref>REF_MAX){
      ref=REF_MAX;
    }
    if(ref<REF_MIN){
      ref=REF_MIN;
    }
    //lee tensión en potenciometro a través de un canal del conversor A/D: [0,1023]
    pot=analogRead(A1);
    //valor actual en grados
    aux=(float)((pot - SENSOR_MEDIO)*POT2GRAD);  //Escalo valores de potenciómetro a grados en los extremos SENSOR_MAX (+90gr) y SENSOR_MIN (-90gr)
    //se pueden agregar filtros digitales a "ref" y "sensor"
    sensor=0.25*(aux+sensor1+sensor2+sensor3);
    sensor3=sensor2;
    sensor2=sensor1;
    sensor1=aux;

    //Calculo de error
    error=ref-sensor;
    
    if(abs(error)<TOLERANCIA)
    {
      motor.run(RELEASE);
    }
    else
    {
      //Controlador PID
      setpoint = ref;     //actualizo el setpoint al controlador PID
      entrada = sensor;   //actualizo la realimentación al controlador PID
      myPID.Compute();    //calculo la acción del controlador PID (escribe la variable "salida")

      //signo positivo o negativo (giro horario o anti-antihorario) o alta impedancia del puente H
      if(salida>=0)
      {
        motor.run(FORWARD);
      }
      else
      {
        motor.run(BACKWARD);
      }
    }
    //mapeo de tension [volt] a PWM de 8 bits 
    pwm_motor=(byte)((abs(salida)+ZONA_MUERTA)*(255.0/SALIDA_MAX));
    motor.setSpeed(pwm_motor);//establece el duty (porcentaje de tiempo en "1") de la señal PWM

    SensorKalman = simpleKalmanFilter.updateEstimate(aux);
 
    bandera_control=0;
  }

  if(bandera_tx==1){
    //sprintf(buffer_tx,"%d,%d",(int)(ref*100.0),(int)(sensor*100.0),(int)(pot*100),(int)(SensorKalman*100));
    sprintf(buffer_tx,"%d,%d,%d,%d",(int)(ref*100.0),(int)(aux*100),(int)(salida*100.0),(int)(SensorKalman*100));
    Serial.println(buffer_tx);
    bandera_tx=0;
  }

  if(bandera_rx==1){
    recvWithEndMarker();
    bandera_rx=0;
  }
}
 
//rutina (función) llamada cada interrupción del timer (1 ms)
void timerIsr()
{
  bandera_control=1; //activa tarea lazo de control cada 2 ms
  contador_tx++;
  contador_rx++;
  
  if(contador_tx>=10){//activa tarea transmisión de datos cada 20 ms
    bandera_tx=1;
    contador_tx=0;
  } 

  if(contador_rx>=5){//polling de recepcion cada 10 ms
    bandera_rx=1;
    contador_rx=0;
  }
}

void recvWithEndMarker()
{
    static byte ndx = 0;
    char endMarker = '\n';
    char rc;
    
    while (Serial.available() > 0)
    {
        
        rc = Serial.read();

        if (rc != endMarker)
        {
            receivedChars[ndx] = rc;
            ndx++;
            if (ndx >= numChars)
            {
                ndx = numChars - 1;
            }
        }
        else
        {
            receivedChars[ndx] = '\0'; // terminate the string
            ndx = 0;
            dataNumber = atof(receivedChars);   // new for this version
        }
    }
}
