//#define  AZ 2
//#define  BZ 3
//#define  VZ 4
//#define  AY 5
//#define  BY 6
//#define  VY 7
//#define  AX 8
//#define  BX 9
//#define  VX 10
//#define  PIN_ENCODER_X 11
//#define  PIN_ENCODER_Z 12
//#define  PIN_ENCODER_Y 13
//
#define  ABORT_OUT      23
#define  COOLER_OUT     25
#define  PISTON_P_OUT   27
#define  PISTON_A_OUT   29
#define  VAR_FREC_OUT   31
//
#define  SPINDLE_IN 24
//#define  ZDIR 26
//#define  YDIR 28
//#define  XDIR 30
//#define  ZSTEP 32
//#define  YSTEP 34
//#define  XSTEP 36
#define  ZLIM_OUT 38
#define  XLIM_OUT 40
#define  YLIM_OUT 42
#define  COOLER_IN 44
#define  HOLD_OUT  46
#define  H_XLIM_OUT 48
#define  H_YLIM_OUT 50
#define  H_ZLIM_OUT 52
//
#define  DIR_IN_EXT 47
#define  FC2_IN 49
#define  PUL_IN_EXT 51
#define  FC1_IN 53



///////////////Variables DEBUG
  int debug_print_pos1=0;
  int debug_print_pos2=0;
  //const uint8_t set_point=192;
///////////////Variables DEBUG
  bool DEBUG_X=0; //Imprimir todas la informacion Normal.  
  bool DEBUG_Y=0; //Imprimir todas la informacion Normal.  
  bool DEBUG_Z=0; //Imprimir todas la informacion Normal.
  int cont_X=0;
  int cont_Y=0;
  int cont_Z=0;
  int pinreset=28;
///////////////Variables SERIAL_USB
  int cont_print_eje=0;
  bool est1=1;
  bool est2=0;
  bool est3=0;
  String Svel="";
  char c1;
  bool rev=0;
  bool enable_recibir_tiempo_driver=0;
  bool enable_recibir_velocidad_minima_driver=0;
  bool enable_recibir_tiempo_driver_corto=0;
  bool enable_recibir_velocidad_minima_driver_corto=0;
  bool enable_aceleracion_corta_constante=1;
  
  uint8_t variable_aceleracion_normal=210; //P210
  uint8_t tiempo_aceleracion_normal=15;    //D15
  uint8_t variable_aceleracion_corto=200;
  uint8_t tiempo_aceleracion_corto=15;
///////////////Variables SERIAL_USB
///////////////Control de velocidad manual
  bool enable_pul_activado=0;
  unsigned long t_manual_motor_velocidad=0;
  bool          E_velocidad0=0;
  bool          E_velocidad1=0;
///////////////Control de velocidad manual

struct Motores { //Motores
  const uint8_t PIN_ENCODER;
  const uint8_t PIN_PUL;
  const uint8_t PIN_DIR;
  const uint8_t PIN_A;
  const uint8_t PIN_B;
  const uint8_t AXIS_LIM;
  const uint8_t PIN_VELOCIDAD;
  volatile int32_t cont_encoder;
  volatile uint8_t cont_pul;
  volatile uint8_t encoder_act;
  volatile bool dir;
  bool dir_act;
  bool deshabilita_movP_motor;
  bool deshabilita_movN_motor;
  bool estado_pin_encoder_act; //borrar buscar
  bool enable_aceleracion_proporcional;
  bool enable_frenar_al_finalizar;
  bool enable_movimiento_motor;
  bool enable_intento_correccion_posicion;  
  int cont_pulsos_interface_por_ciclo;////////////////////////*******debug_pul
  int cont_pasos_encoder_por_ciclo;////////////////////////*******debug_encoder
  int diferencia_entre_encoder_e_interfase; // borrar
  int tiempo_encendido_motor;
  int cont_pasos_encoder_correctos;
  int pasos_encoder_inerciales;
  int cont_pasos_encoder_en_movimiento;
  int encoder_corregir; // borrar
  int cont_motor_desconectado;
  int variable_velocidad_movimiento_motor;
  int pasos_encoder_faltantes;
  int setpoint_posicion_act;
  int setpoint_posicion_prev;
  int setpoint_encoder;
  int diferencia_posicion_1; //borrar
  int cont_pulsos_interfase_extra;
  unsigned long t_pul_inicio;
  unsigned long t_pul;
  unsigned long t_encoder_inicio;
  unsigned long t_encoder;
  unsigned long t_inicio_mov;
  unsigned long t_mov;
  unsigned long t_fin_mov;
  unsigned long t_inercia;  
  unsigned long t_inactivo;
  unsigned long variable_cont_aceleracion;
  uint8_t est_motor;
};
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
struct MANDO_VAR {
bool EP1; bool EP2; bool EA0;  bool EA1; bool EA2; bool EA3; bool ES0; bool ES1; bool ES2; bool ES3;
bool rev; uint8_t enviando_comando_tipo;
bool enable_enviado_peticion_completo; uint8_t peticion_recibida;
bool enviado_accion_completo; uint8_t accion_recibida; uint8_t accion_num_recibida;
bool enviado_velocidad_completo; uint8_t velocidad_recibida;
bool recibido_nuevo_comando;
};
MANDO_VAR MANDO={0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,  0,0,0,0,0};
bool enable_cambiar_modo=0;
bool  cambiar_a_modo=0; // volver a modo normal 0/ manejar manualmente1
bool  modo_act=0; // en que modo estoy Normal o Manual
uint8_t estado_act=0;
uint8_t error_act=0;
uint8_t valvulas_act=0;
bool enable_habilitar_movimiento_desde_mando=0;
bool enable_obligar_desactivacion_motores=0;
bool enable_mover_ejes_manualmente=0;
uint8_t ejes_para_mover_manualmente=0;
bool enable_cambiar_velocidad=0;
uint8_t velocidad_MANDO_act=255;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
struct Motores_Posicion{
  int num_pasos_por_mm;//=96;  
  int pul_limite_negativo_max;//=15;  //limite en milimetros
  int pul_limite_positivo_max;//=275; //limite en milimetros
  int pul_posicion_home;      //=110;
  int pul_margen_seguro_en_limites;//=10; //limite en milimetros
  int pul_posicion_eje_act;//=0;
};

struct Motores_Debug { //Motor_Errores1
  int error_cont_pul_mov0;
  int error_cont_pul_mov1;
  int error_cont_encoder_mov0;
  int error_cont_encoder_mov1;
  unsigned long t1;
  unsigned long t2;
  unsigned long t3;
  volatile int32_t encoder_debug;
  volatile int32_t pul_interf_debug;
  int cont_pasos_encoder_totales;
  int cont_pulsos_interface_totales;
};

struct Motores_Errores { //Motor_Errores1
  int error_posicion_motor;
  int error_pulsos_interfase_extra;
  int error_pasos_encoder_perdidos;
};
Motores_Posicion Ep1={0 ,130, 25,10,96,0}; // verificar
Motores_Posicion Ep2={1440,26400,10560,10,960,0};//{15,275,110,10,96,0};
Motores_Posicion Ep3={10,100, 30,10,96,0}; // verificar
Motores_Errores Me1={0,0,0};
Motores_Errores Me2={0,0,0};
Motores_Errores Me3={0,0,0};
Motores_Debug Md1={0,0,0,0,0,0,0,0,0,0,0};
Motores_Debug Md2={0,0,0,0,0,0,0,0,0,0,0};
Motores_Debug Md3={0,0,0,0,0,0,0,0,0,0,0};

Motores M1= {
            11,  //PIN_ENCODER    12
            36, //PIN_PUL         32
            30, //PIN_DIR;        26
            8,  //PIN_A;          2
            9,  //PIN_B;          3
            40, //AXIS_LIM;       38
            10,  //PIN_VELOCIDAD; 4
            0,  //cont_encoder;   
            0,  //cont_pul;
            0,  //encoder_act;
            false,//dir
            false,//dir_act;
            false,//deshabilita_movP_motor;
            false,//deshabilita_movN_motor
            false,//bool estado_pin_encoder_act;
            false,//bool enable_aceleracion_proporcional;
            false,//bool enable_frenar_al_finalizar;
            false,//bool enable_movimiento_motor;
            false,//bool enable_intento_correccion_posicion;  
            0,//int cont_pulsos_interface_por_ciclo;////////////////////////*******debug_pul
            0,//int cont_pasos_encoder_por_ciclo;////////////////////////*******debug_encoder
            0,//int diferencia_entre_encoder_e_interfase;
            0,//int tiempo_encendido_motor;
            0,//int cont_pasos_encoder_correctos;
            0,//int pasos_encoder_inerciales;
            0,//int cont_pasos_encoder_en_movimiento;
            0,//encoder_corregir;
            0,//int cont_motor_desconectado;
            0,//int variable_velocidad_movimiento_motor;
            0,//int pasos_encoder_faltantes;
            0,//int setpoint_posicion_act;
            0,//int setpoint_posicion_prev;
            0,//int setpoint_encoder;
            0,//int diferencia_posicion_1;
            0,//int cont_pulsos_interfase_extra;
            0,//unsigned long t_pul_inicio=0;
            0,//unsigned long t_pul=0;
            0,//unsigned long t_encoder_inicio=0;
            0,//unsigned long t_encoder=0;
            0,//unsigned long t_inicio_mov=0;
            0,//unsigned long t_mov=0;
            0,//unsigned long t_fin_mov=0;
            0,//unsigned long t_inercia=0;  
            0,//unsigned long t_inactivo=0;
            0,//unsigned long variable_cont_aceleracion;
            0 //uint8_t est_motor;
            };   


Motores M2= {
            13,  //PIN_ENCODER_Y
            34, //PIN_PUL_Y
            28, //PIN_DIR;_Y
            5,  //PIN_A_Y;
            6,  //PIN_B_Y;
            42,  //AXIS_LIM_Y;
            7, //PIN_VELOCIDAD_Y;
            0,  //cont_encoder;
            0,  //cont_pul;
            0,  //encoder_act;
            false,//dir
            false,//dir_act;
            false,//deshabilita_movP_motor;
            false,//deshabilita_movN_motor
            false,//bool estado_pin_encoder_act;
            false,//bool enable_aceleracion_proporcional;
            false,//bool enable_frenar_al_finalizar;
            false,//bool enable_movimiento_motor;
            false,//bool enable_intento_correccion_posicion;  
            0,//int cont_pulsos_interface_por_ciclo;////////////////////////*******debug_pul
            0,//int cont_pasos_encoder_por_ciclo;////////////////////////*******debug_encoder            
            0,//int diferencia_entre_encoder_e_interfase;
            0,//int tiempo_encendido_motor;
            0,//int cont_pasos_encoder_correctos;
            0,//int pasos_encoder_inerciales;
            0,//int cont_pasos_encoder_en_movimiento;
            0,//encoder_corregir;
            0,//int cont_motor_desconectado;
            0,//int variable_velocidad_movimiento_motor;
            0,//int pasos_encoder_faltantes;
            0,//int setpoint_posicion_act;
            0,//int setpoint_posicion_prev;
            0,//int setpoint_encoder;
            0,//int diferencia_posicion_1;
            0,//int cont_pulsos_interfase_extra;            
            0,//unsigned long t_pul_inicio=0;
            0,//unsigned long t_pul=0;
            0,//unsigned long t_encoder_inicio=0;
            0,//unsigned long t_encoder=0;
            0,//unsigned long t_inicio_mov=0;
            0,//unsigned long t_mov=0;
            0,//unsigned long t_fin_mov=0;
            0,//unsigned long t_inercia=0;  
            0,//unsigned long t_inactivo=0;
            0,//unsigned long variable_cont_aceleracion;
            0 //uint8_t est_motor;
            };   

Motores M3= {
            12,  //PIN_ENCODER  11
            32, //PIN_PUL       36
            26, //PIN_DIR;      30
            2,  //PIN_A;        8
            3,  //PIN_B;        9
            38,  //AXIS_LIM;    40
            4, //PIN_VELOCIDAD; 10
            0,  //cont_encoder;
            0,  //cont_pul;
            0,  //encoder_act;
            false,//dir
            false,//dir_act;
            false,//deshabilita_movP_motor;
            false,//deshabilita_movN_motor
            false,//bool estado_pin_encoder_act;
            false,//bool enable_aceleracion_proporcional;
            false,//bool enable_frenar_al_finalizar;
            false,//bool enable_movimiento_motor;
            false,//bool enable_intento_correccion_posicion;  
            0,//int cont_pulsos_interface_por_ciclo;////////////////////////*******debug_pul
            0,//int cont_pasos_encoder_por_ciclo;////////////////////////*******debug_encoder
            0,//int diferencia_entre_encoder_e_interfase;
            0,//int tiempo_encendido_motor;
            0,//int cont_pasos_encoder_correctos;
            0,//int pasos_encoder_inerciales;
            0,//int cont_pasos_encoder_en_movimiento;
            0,//encoder_corregir;
            0,//int cont_motor_desconectado;
            0,//int variable_velocidad_movimiento_motor;
            0,//int pasos_encoder_faltantes;
            0,//int setpoint_posicion_act;
            0,//int setpoint_posicion_prev;
            0,//int setpoint_encoder;
            0,//int diferencia_posicion_1;
            0,//int cont_pulsos_interfase_extra;            
            0,//unsigned long t_pul_inicio=0;
            0,//unsigned long t_pul=0;
            0,//unsigned long t_encoder_inicio=0;
            0,//unsigned long t_encoder=0;
            0,//unsigned long t_inicio_mov=0;
            0,//unsigned long t_mov=0;
            0,//unsigned long t_fin_mov=0;
            0,//unsigned long t_inercia=0;  
            0,//unsigned long t_inactivo=0;
            0,//unsigned long variable_cont_aceleracion;
            0 //uint8_t est_motor;
            };   

void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);
  pinMode(ABORT_OUT, OUTPUT);
  pinMode(ZLIM_OUT, OUTPUT);
  pinMode(YLIM_OUT, OUTPUT);
  pinMode(XLIM_OUT, OUTPUT);
  pinMode(HOLD_OUT, OUTPUT);
  
  pinMode(H_XLIM_OUT, INPUT);
  pinMode(H_YLIM_OUT, INPUT);
  pinMode(H_ZLIM_OUT, INPUT);
  pinMode(SPINDLE_IN, INPUT);
  pinMode(COOLER_IN, INPUT);
  pinMode(PUL_IN_EXT, INPUT);
  pinMode(DIR_IN_EXT, INPUT);
  pinMode(FC1_IN, INPUT);
  pinMode(FC2_IN, INPUT);
  
  digitalWrite(ABORT_OUT,1);
  digitalWrite(ZLIM_OUT,1);
  digitalWrite(YLIM_OUT,1);
  digitalWrite(XLIM_OUT,1);
  digitalWrite(HOLD_OUT,1);

  /////////////////////////OUTPUT_RELE///////////////////////////////////////
  pinMode(COOLER_OUT, OUTPUT);
  pinMode(PISTON_P_OUT, OUTPUT);
  pinMode(PISTON_A_OUT, OUTPUT);
  pinMode(VAR_FREC_OUT, OUTPUT);
  digitalWrite(COOLER_OUT,1);
  digitalWrite(PISTON_P_OUT,1);
  digitalWrite(PISTON_A_OUT,1);
  digitalWrite(VAR_FREC_OUT,1);
  
  //////////////////////////////////MOTOR_X///////////////////////////////////
  pinMode(M1.PIN_ENCODER, INPUT_PULLUP);  // Pin del encoder de motor
  pinMode(M1.PIN_PUL, INPUT_PULLUP);      // Pin del Pulso de la interfase
  pinMode(M1.PIN_DIR, INPUT_PULLUP);      // Pin del Direccion de la interfase
  pinMode(M1.PIN_A, OUTPUT);              // Pin del Driver A
  pinMode(M1.PIN_B, OUTPUT);              // Pin del Driver B
  pinMode(M1.AXIS_LIM, INPUT_PULLUP);     // Pin Fin de Carrera del Motor
  pinMode(M1.PIN_VELOCIDAD, OUTPUT);     // Pin PWM para el Control de Velocidad del Motor
  attachInterrupt(digitalPinToInterrupt(M1.PIN_ENCODER), inter_encoder_X1, FALLING); //PIN2 
  attachInterrupt(digitalPinToInterrupt(M1.PIN_PUL), inter_pulsos_X1 , FALLING); //PIN3
  digitalWrite(M1.PIN_A,true);
  digitalWrite(M1.PIN_B,true);
  //////////////////////////////////MOTOR_Y///////////////////////////////////
  pinMode(M2.PIN_ENCODER, INPUT_PULLUP);  // Pin del encoder de motor
  pinMode(M2.PIN_PUL, INPUT_PULLUP);      // Pin del Pulso de la interfase
  pinMode(M2.PIN_DIR, INPUT_PULLUP);      // Pin del Direccion de la interfase
  pinMode(M2.PIN_A, OUTPUT);              // Pin del Driver A
  pinMode(M2.PIN_B, OUTPUT);              // Pin del Driver B
  pinMode(M2.AXIS_LIM, INPUT_PULLUP);     // Pin Fin de Carrera del Motor
  pinMode(M2.PIN_VELOCIDAD, OUTPUT);     // Pin PWM para el Control de Velocidad del Motor
  attachInterrupt(digitalPinToInterrupt(M2.PIN_ENCODER), inter_encoder_Y1, FALLING); //PIN6 
  attachInterrupt(digitalPinToInterrupt(M2.PIN_PUL), inter_pulsos_Y1 , FALLING); //PIN7
  digitalWrite(M2.PIN_A,true);
  digitalWrite(M2.PIN_B,true);
  //////////////////////////////////MOTOR_Z///////////////////////////////////
  pinMode(M3.PIN_ENCODER, INPUT_PULLUP);  // Pin del encoder de motor
  pinMode(M3.PIN_PUL, INPUT_PULLUP);      // Pin del Pulso de la interfase
  pinMode(M3.PIN_DIR, INPUT_PULLUP);      // Pin del Direccion de la interfase
  pinMode(M3.PIN_A, OUTPUT);              // Pin del Driver A
  pinMode(M3.PIN_B, OUTPUT);              // Pin del Driver B
  pinMode(M3.AXIS_LIM, INPUT_PULLUP);     // Pin Fin de Carrera del Motor
  pinMode(M3.PIN_VELOCIDAD, OUTPUT);     // Pin PWM para el Control de Velocidad del Motor
  attachInterrupt(digitalPinToInterrupt(M3.PIN_ENCODER), inter_encoder_Z1, FALLING); //PIN6 
  attachInterrupt(digitalPinToInterrupt(M3.PIN_PUL), inter_pulsos_Z1 , FALLING); //PIN7
  digitalWrite(M3.PIN_A,true);
  digitalWrite(M3.PIN_B,true);
  //////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////
  Serial.println(F("Modulo Controlador NCL y NCM SOFTWARE_V1.0"));
  Serial.println(F("Motor: 96pasos = 1 milimetro | VelocidadMax = 100 mm/min | aceleracion maxima = 6 mm/sec^2"));
  delay(500);
  imprimir_datos_ciclo_Y();
  M1.encoder_act=0;
  Md1.encoder_debug=0;
  M2.encoder_act=0;
  Md2.encoder_debug=0;
  M3.encoder_act=0;
  Md3.encoder_debug=0;
}

void loop(){
    Control_Principal();
    if (Serial1.available() > 0) {
      Recepcion_Dato_MANDO();
    }
    if(MANDO.recibido_nuevo_comando){
      MANDO.recibido_nuevo_comando=0;
      Respuesta_a_comandos_MANDO();
    }
    if(!enable_habilitar_movimiento_desde_mando){ // solo se habilita cuandoo M1.encoder_act==0 && M2.encoder_act==0 && M3.encoder_act==0
      control_a_pasos_servomotor_eje_X();
      control_a_pasos_servomotor_eje_Y();
      control_a_pasos_servomotor_eje_Z();
    }
      //recibir_datos_serial_USB();
}

void enviar_a_CP(String comando){
  Serial1.println(comando);
}

void recibir_datos_serial_USB(){
  if (Serial.available()){
    c1 = Serial.read();
    if(isDigit(c1)){
      if(est1){
        Svel+=c1;
        rev=1;
      }
      if(est2){
        Svel+=c1;
        rev=1;
        if(enable_recibir_tiempo_driver){
          tiempo_aceleracion_normal=String(Svel).toInt();
          Serial.println("");
          Serial.print("D");
          Serial.println(tiempo_aceleracion_normal);
          Svel="";
          est2=0;
          est1=1;
          rev=0;
          enable_recibir_tiempo_driver=0;
        }
        if(enable_recibir_tiempo_driver_corto){
          tiempo_aceleracion_corto=String(Svel).toInt();
          Serial.println("");
          Serial.print("d");
          Serial.println(tiempo_aceleracion_corto);
          Svel="";
          est2=0;
          est1=1;
          rev=0;
          enable_recibir_tiempo_driver_corto=0;
        }
      }
      if(est3){
        Svel+=c1;
        rev=1;
        if(enable_recibir_velocidad_minima_driver){
          variable_aceleracion_normal=String(Svel).toInt();
          Svel="";
          Serial.println("");
          Serial.print("P");
          Serial.println(variable_aceleracion_normal);
          //enable_recibir_velocidad_minima_driver=0;
        }
        if(enable_recibir_velocidad_minima_driver_corto){
          variable_aceleracion_corto=String(Svel).toInt();
          Svel="";
          Serial.println("");
          Serial.print("p");
          Serial.println(variable_aceleracion_corto);
          //enable_recibir_velocidad_minima_driver_corto=0;
        }
        Svel=""; //reset variable
      }
      if(est1 && rev){
        est1=0;
        est2=1;
        rev=0;
      }
      if(est2 && rev){
        est2=0;
        est3=1;
        rev=0;
      }
      if(est3 && rev){
        est3=0;
        est1=1;
        rev=0;
      }
    }else{
      if(c1=='D'){ // tiempo de funcionamiento del Drive
        enable_recibir_tiempo_driver=1;
        Serial.println("");
        Serial.print(F("DRIVER_ENABLE"));
      }
      if(c1=='d'){ // tiempo de funcionamiento del Drive
        enable_recibir_tiempo_driver_corto=1;
        Serial.println("");
        if(enable_aceleracion_corta_constante){
          Serial.print(F("CORTO_Const"));
        }else{
          Serial.print(F("CORTO_Propor"));
        }
      }
      if(c1=='e'){ // tiempo de funcionamiento del Drive
        //enable_aceleracion_corta_constante=!enable_aceleracion_corta_constante;
        Serial.println("");
        Serial.print("|PX|");
        Serial.print(Ep1.pul_posicion_eje_act);
        Serial.print("|Er|");
        Serial.print(Me1.error_posicion_motor);
        Serial.print("|PY|");
        Serial.print(Ep2.pul_posicion_eje_act);
        Serial.print("|Er|");
        Serial.print(Me2.error_posicion_motor);
        Serial.print("|PZ|");
        Serial.print(Ep3.pul_posicion_eje_act);
        Serial.print("|Er|");
        Serial.print(Me3.error_posicion_motor);
      }
      if(c1=='P'){
        enable_recibir_velocidad_minima_driver=1;
        enable_recibir_velocidad_minima_driver_corto=0;
        Serial.println("");
        Serial.print(F("PWM_ENA"));
      }
      if(c1=='p'){
        enable_recibir_velocidad_minima_driver_corto=1;
        enable_recibir_velocidad_minima_driver=0;
        Serial.println("");
        Serial.print(F("PWM_SHORT"));
      }
      if(c1=='c'){
          DEBUG_X=0; //Imprimir todas la informacion Normal.  
          DEBUG_Y=0; //Imprimir todas la informacion Normal.  
          DEBUG_Z=0; //Imprimir todas la informacion Normal.
          if(cont_print_eje==0)DEBUG_X=1;
          if(cont_print_eje==1)DEBUG_Y=1;
          if(cont_print_eje==2)DEBUG_Z=1;
          cont_print_eje+=1;
          if(cont_print_eje>2){
            cont_print_eje=0;
          }
      }
      if(c1=='h'){
        Serial.println("");
        Serial.print(Md2.encoder_debug);
        Serial.print("|");
        Serial.println(Md2.pul_interf_debug);
        M1.deshabilita_movP_motor=0;
        M1.deshabilita_movN_motor=0;
        M2.deshabilita_movP_motor=0;
        M2.deshabilita_movN_motor=0;
        M3.deshabilita_movP_motor=0;
        M3.deshabilita_movN_motor=0;        
      }
      if(c1=='a'){ // tiempo de funcionamiento del Drive
        Serial.println("");
        Serial.print("|AX");
        Serial.print("|T|");
        Serial.print(Md1.cont_pasos_encoder_totales);
        Serial.print("|");
        Serial.print(Md1.cont_pasos_encoder_totales-Md1.cont_pulsos_interface_totales);    
        Serial.print("|t|");
        Serial.print(Md1.cont_pulsos_interface_totales);
        Serial.print("|");
        Serial.print("AY");
        Serial.print("|T|");
        Serial.print(Md2.cont_pasos_encoder_totales);
        Serial.print("|");
        Serial.print(Md2.cont_pasos_encoder_totales-Md2.cont_pulsos_interface_totales);    
        Serial.print("|t|");
        Serial.print(Md2.cont_pulsos_interface_totales);
        Serial.print("|");
        Serial.print("AZ");
        Serial.print("|T|");
        Serial.print(Md3.cont_pasos_encoder_totales);
        Serial.print("|");
        Serial.print(Md3.cont_pasos_encoder_totales-Md3.cont_pulsos_interface_totales);    
        Serial.print("|t|");
        Serial.print(Md3.cont_pulsos_interface_totales);
        Serial.print("|");        
        Serial.print("|X");
        Serial.print("|E|");
        Serial.print(Md1.cont_pasos_encoder_totales);
        Serial.print("|P|");
        Serial.print(Md1.cont_pulsos_interface_totales);
        Serial.print("|Y");
        Serial.print("|E|");
        Serial.print(Md2.cont_pasos_encoder_totales);
        Serial.print("|P|");
        Serial.print(Md2.cont_pulsos_interface_totales);
        Serial.print("|Z");
        Serial.print("|E|");
        Serial.print(Md3.cont_pasos_encoder_totales);
        Serial.print("|P|");
        Serial.print(Md3.cont_pulsos_interface_totales);
      }
    }
  }
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////MOTOR_X/////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////MOTOR_X/////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////MOTOR_X/////////////////////////////////////////////////////////////
void control_a_pasos_servomotor_eje_X(){
  if(M1.est_motor==0){ // ServoMotor Parado
    if(M1.encoder_act>0){ // ServoMotor Inercia
      estado_motor_inercia_X0();
    }
    if(M1.cont_pul==1){// ServoMotor Comenzar Movimiento
      estado_comenzar_movimiento_X1();
    }else{
      if(M1.cont_pul>1){
        estado_mov_X3();
        estado_comenzar_movimiento_X1();
      }
    }
  }
  if(M1.est_motor==1){
    if(M1.cont_pul>0){// Pulso de Interfase Extra
      estado_mov_X4();
    }
    M1.t_mov=millis()-M1.t_inicio_mov;
    aceleracion_motor_proporcional_X();
    verificar_encoder_tiempo_en_movimiento_X();
  }
  if(M1.est_motor==2){
    M1.t_inercia=millis()-M1.t_fin_mov;
    frenado_motor_X();
  }
}
void estado_motor_inercia_X0(){
  Md1.t2=millis()-M1.t_inicio_mov;
  //M1.pasos_encoder_inerciales+=M1.encoder_act;
  if(M1.pasos_encoder_faltantes>0){
    M1.pasos_encoder_faltantes-=M1.encoder_act;
    M1.cont_pasos_encoder_correctos+=M1.encoder_act;
  }else{
    M1.pasos_encoder_inerciales+=M1.encoder_act;
  }
  M1.cont_pasos_encoder_por_ciclo+=M1.encoder_act;
  M1.encoder_act=0;
}
void estado_comenzar_movimiento_X1(){
  //verificacion_motor_X();
  //////////////////////////////////////////////////////////////////*********************************************
  if(M1.deshabilita_movP_motor || M1.deshabilita_movN_motor){
    Me1.error_pulsos_interfase_extra+=M1.cont_pulsos_interfase_extra;
    encerado_variables_ciclo_X();
    M1.setpoint_posicion_act=0;
  }
  //////////////////////////////////////////////////////////////////*********************************************
  actualizar_posicion_eje_X();
  M1.pasos_encoder_inerciales+=M1.encoder_act;
  actualizar_setpoint_X();
       imprimir_datos_ciclo_inercia_X();
  inicializacion_variables_ciclo_X();
  encerado_variables_ciclo_X();
  validacion_movimiento_encoder_X();
  /////////////////////////////////////////////////////////////////////
  if(M1.enable_movimiento_motor){
    if(M1.variable_velocidad_movimiento_motor==1){ //Acelera proporcionalmente
      M1.enable_frenar_al_finalizar=1;
      M1.enable_aceleracion_proporcional=1;
      M1.est_motor=1;
      M1.tiempo_encendido_motor=15; // desde 4-15 [ms] milisegundos
      //M1.tiempo_encendido_motor=tiempo_aceleracion_normal; // desde 4-15 [ms] milisegundos
      mover_motor_X(M1.dir_act, 255);
    }
    if(M1.variable_velocidad_movimiento_motor==2){
      M1.est_motor=1;      
      M1.tiempo_encendido_motor=15;
      //M1.tiempo_encendido_motor=map(M1.t_pul,4,15,9,7);
      mover_motor_X(M1.dir_act, 255);
    }
  }else{
    M1.variable_velocidad_movimiento_motor=0;
    M1.tiempo_encendido_motor=0;
    //estado_mov_X5();
    digitalWrite(M1.PIN_A,true);
    digitalWrite(M1.PIN_B,true);
    M1.t_fin_mov=millis();
    Md1.t1=millis()-M1.t_inicio_mov;
    M1.enable_aceleracion_proporcional=0;
    M1.cont_pasos_encoder_en_movimiento=0;
    //M1.dir_prev=M1.dir_act;
    M1.est_motor=2;
    imprimir_datos_ciclo_X();
  }
}
void estado_mov_X3(){
  if(M1.dir){
    M1.cont_pulsos_interfase_extra+=M1.cont_pul;
    M1.cont_pulsos_interfase_extra-=1;
  }else{
    M1.cont_pulsos_interfase_extra-=M1.cont_pul;
    M1.cont_pulsos_interfase_extra+=1;
  }
}
void estado_mov_X4(){
  if(M1.dir==M1.dir_act){
    if(M1.enable_aceleracion_proporcional){
      M1.enable_aceleracion_proporcional=0;
      M1.enable_frenar_al_finalizar=0;
      aceleracion_motor_instantanea_X();
    }
    if(M1.cont_pul>1){
      Serial.print(F("ERROR_ demasiados_PULSOS_X"));
    }
    M1.tiempo_encendido_motor=14;
    M1.cont_pulsos_interface_por_ciclo+=M1.cont_pul;////////////////////////*******debug_pul
    M1.cont_pul=0;
    M1.setpoint_encoder+=1;
    if(M1.setpoint_encoder>2){
      M1.setpoint_encoder=2;
      if(M1.dir){
        M1.cont_pulsos_interfase_extra+=1;
      }else{
        M1.cont_pulsos_interfase_extra-=1;
      }
    }
  }else{
    if(M1.dir){
      M1.cont_pulsos_interfase_extra+=1;
    }else{
      M1.cont_pulsos_interfase_extra-=1;
    }
  }
}
void aceleracion_motor_proporcional_X(){
  if(M1.enable_aceleracion_proporcional){
    if(M1.t_mov!=M1.variable_cont_aceleracion){
      analogWrite(M1.PIN_VELOCIDAD,255-(M1.t_mov*(variable_aceleracion_corto/tiempo_aceleracion_corto)));
      M1.variable_cont_aceleracion=M1.t_mov;
    }
  }
}

void aceleracion_motor_instantanea_X(){
  mover_motor_X(M1.dir_act, 255);
}
void verificar_encoder_tiempo_en_movimiento_X(){
  if(M1.enable_movimiento_motor){
    if(M1.encoder_act==M1.setpoint_encoder){
      estado_mov_X5();
    }else{
      if(M1.t_mov>M1.tiempo_encendido_motor){
        if(M1.encoder_act<M1.setpoint_encoder){ // encoder falta de moverse
          M1.pasos_encoder_faltantes=M1.setpoint_encoder-M1.encoder_act;
        }
        if(M1.encoder_act>M1.setpoint_encoder){ // encoder se movio de mas
          M1.pasos_encoder_inerciales+=M1.encoder_act-M1.setpoint_encoder;
        }
        estado_mov_X5();
      }else{
        if(M1.encoder_act>M1.setpoint_encoder){
          M1.pasos_encoder_inerciales+=M1.encoder_act-M1.setpoint_encoder;
          estado_mov_X5();
        }
      }
    }
  }else{
    estado_mov_X5();
  }
}

void estado_mov_X5(){
  digitalWrite(M1.PIN_A,true);
  digitalWrite(M1.PIN_B,true);
  M1.cont_pasos_encoder_correctos=M1.encoder_act;
  M1.t_fin_mov=millis();
  Md1.t1=millis()-M1.t_inicio_mov;
  M1.enable_aceleracion_proporcional=0;
  M1.cont_pasos_encoder_en_movimiento=M1.encoder_act;// con ese puedo saber si recibi o fue por tiempo
  M1.cont_pasos_encoder_por_ciclo+=M1.encoder_act;////////////////////////*******debug_encoder
  M1.encoder_act=0;
  //M1.dir_prev=M1.dir_act;
  M1.est_motor=2;
  imprimir_datos_ciclo_X();
}
void frenado_motor_X(){
  if(M1.enable_frenar_al_finalizar){
    if(M1.t_inercia==1){
      analogWrite(M1.PIN_VELOCIDAD,255);
      if(!M1.dir_act){
        digitalWrite(M1.PIN_A,true);
        digitalWrite(M1.PIN_B,false);
      }else{
        digitalWrite(M1.PIN_A,false);
        digitalWrite(M1.PIN_B,true);
      }
    }
    if(M1.t_inercia>=2){
      digitalWrite(M1.PIN_A,true);
      digitalWrite(M1.PIN_B,true);
      M1.enable_frenar_al_finalizar=0;
      M1.est_motor=0;
    }
  }else{
    M1.est_motor=0;
  }
}
void validacion_movimiento_encoder_X(){
  if((M1.dir && M1.deshabilita_movP_motor) || (!M1.dir && M1.deshabilita_movN_motor)){
      M1.setpoint_encoder=0;      
      M1.enable_movimiento_motor=0;
  }else{

      
    if(M1.setpoint_posicion_act==0){
      M1.setpoint_encoder=1;      
      M1.enable_movimiento_motor=1;
    }else{
      if(M1.dir_act){
        if(M1.setpoint_posicion_act>0){
          M1.setpoint_posicion_act-=1;
          M1.enable_movimiento_motor=0;
          M1.setpoint_encoder=0;
        }else{
          if(M1.setpoint_posicion_act<0){
            M1.setpoint_encoder=2;
            M1.setpoint_posicion_act+=1;
            M1.enable_intento_correccion_posicion=1;///////////////////////*/*/*/*
          }else{
            M1.setpoint_encoder=1;
          }
          M1.enable_movimiento_motor=1;
        }
      }else{
        if(M1.setpoint_posicion_act<0){
          M1.setpoint_posicion_act+=1;
          M1.enable_movimiento_motor=0;
          M1.setpoint_encoder=0;
        }else{
          if(M1.setpoint_posicion_act>0){
            M1.setpoint_encoder=2;
            M1.setpoint_posicion_act-=1;
            M1.enable_intento_correccion_posicion=1;///////////////////////*/*/*/*
          }else{
            M1.setpoint_encoder=1;
          }
          M1.enable_movimiento_motor=1;
        }
      }
    }
    if(M1.enable_movimiento_motor){
      if(M1.t_pul>15 && M1.setpoint_encoder==1){
        M1.variable_velocidad_movimiento_motor=1;
      }else{
        M1.variable_velocidad_movimiento_motor=2;
      }
    }else{
      M1.variable_velocidad_movimiento_motor=0;
    }
  
  }
}
/////////////////////////////////////////////////==INTERRUPCIONES==//////////////////////
void inter_encoder_X1(){
  M1.encoder_act +=1;
  Md1.encoder_debug+=1;
}

void inter_pulsos_X1(){
  M1.dir=digitalRead(M1.PIN_DIR);
  M1.cont_pul += 1;
  Md1.pul_interf_debug+=1;
}
//////////////////////////////////////////////////***PRINT****///////////////////////////
void imprimir_datos_ciclo_X(){
  if(DEBUG_X){
    Serial.println("");
    cont_X++;
    if(cont_X>9){
      Serial.print(cont_X);
    }else{
      Serial.print("0");
      Serial.print(cont_X);
    }
    if(cont_X>95){
      cont_X=0;
    }
    if(M1.dir_act){
      Serial.print("|P");
    }else{
      Serial.print("|N");
    }
    if(M1.t_pul>99){
      Serial.print("99"); // Tiempo entre pulsos
    }else{
      if(M1.t_pul>9){
        Serial.print(M1.t_pul); // Tiempo entre pulsos
      }else{
        Serial.print("0");
        Serial.print(M1.t_pul);
      }
    }
    Serial.print("|D");
    if(M1.tiempo_encendido_motor>9){
      Serial.print(M1.tiempo_encendido_motor);
    }else{
      Serial.print("0");
      Serial.print(M1.tiempo_encendido_motor);
    }
    Serial.print("|t1|");
    if(Md1.t1>9){
      Serial.print(Md1.t1);
    }else{
      Serial.print("0");
      Serial.print(Md1.t1);
    }
    Serial.print("|");
    Serial.print(M1.cont_pasos_encoder_en_movimiento); //Pulsos recibidos asta estadoV5
    Serial.print("|SP|");
    Serial.print(M1.setpoint_encoder);
    //Serial.print("|");
 }
}
void imprimir_datos_ciclo_inercia_X(){
  if(DEBUG_X){
//    Serial.print("|F|");
//    Serial.print(M1.pasos_encoder_faltantes); ////print_inercia
    Serial.print("|");
    Serial.print(M1.cont_pasos_encoder_correctos); ////print_inercia
    Serial.print("|I|");
    Serial.print(M1.pasos_encoder_inerciales);////print_inercia
//    Serial.print("|M|");
    Serial.print("|t2|");     //datos ciclo anterior
    if(M1.pasos_encoder_inerciales>0){
      if(Md1.t2>9){
        Serial.print(Md1.t2);
      }else{
        Serial.print("0");
        Serial.print(Md1.t2);
      }
    }else{
      Serial.print("  ");
    }
    //////////////////DEBUG//////////////////////
    //Serial.print("|");
    Serial.print("|");
    Serial.print(M1.enable_intento_correccion_posicion); ////print_inercia
    Serial.print("|P_X|");
    Serial.print(M1.cont_pulsos_interfase_extra); //print_inercia
//    Serial.print("|C|");  
//    Serial.print(Md1.cont_pasos_encoder_totales);
//    Serial.print("|");
//    Serial.print(Md1.cont_pasos_encoder_totales-Md1.cont_pulsos_interface_totales);    
//    Serial.print("|p|");
//    Serial.print(Md1.cont_pulsos_interface_totales);
//    Serial.print("|");
    Serial.print("|L|");
    Serial.print(M1.cont_pasos_encoder_por_ciclo);////////////////////////*******debug_encoder //print_inercia
    Serial.print("|");
    Serial.print(M1.cont_pulsos_interface_por_ciclo);////////////////////////*******debug_pul //print_inercia
    Serial.print("|");
    Serial.print(M1.cont_pasos_encoder_por_ciclo-M1.cont_pulsos_interface_por_ciclo);////////////////////////*******debug_encoder
    Serial.print("|E|");
    Serial.print(Me1.error_posicion_motor);
    Serial.print("|S|");
    Serial.print(Ep1.pul_posicion_eje_act);
  }
}
//////////////////////////////////////////////////***PRINT****///////////////////////////

void verificacion_motor_X(){
  if(Md1.encoder_debug==0){
    M1.cont_motor_desconectado++;
    if(M1.cont_motor_desconectado>2){
      Serial.println("MotorNoResponde_X");
      //error_motor_no_responde=1;
      M1.cont_motor_desconectado=0;
      if(M1.dir_act){
        M1.deshabilita_movP_motor=1;
        Serial.print("P");
      }else{
        M1.deshabilita_movN_motor=1;
        Serial.print("N");
      }
      Serial.print("Requiere Calibrado_X");
       //habilitar fin de carrera
       //enviar stop y pausa
       //Recomendar Calibrar
    }
  }else{
    M1.cont_motor_desconectado=0;
  }
}

void mover_motor_X(bool direccion, uint8_t velocidad){
  analogWrite(M1.PIN_VELOCIDAD,velocidad);
  if(direccion){
    digitalWrite(M1.PIN_A,true);
    digitalWrite(M1.PIN_B,false);
  }else{
    digitalWrite(M1.PIN_A,false);
    digitalWrite(M1.PIN_B,true);
  }
}

void actualizar_posicion_eje_X(){
  M1.cont_pulsos_interface_por_ciclo+=M1.cont_pul;////////////////////////*******debug_pul
  M1.cont_pasos_encoder_por_ciclo+=M1.encoder_act;////////////////////////*******debug_encoder
//  Md1.cont_pasos_encoder_totales+=Md1.encoder_debug;
//  Md1.cont_pulsos_interface_totales+=Md1.pul_interf_debug;
  if(M1.cont_pulsos_interface_por_ciclo!=Md1.pul_interf_debug){
    Serial.println(F("no_concuerda_pul_interfase_X "));
    Serial.print(M1.cont_pulsos_interface_por_ciclo);
    Serial.print("|");
    Serial.print(Md1.pul_interf_debug);
    Serial.print("|");
  }
  if(M1.cont_pasos_encoder_por_ciclo!=Md1.encoder_debug){
    Serial.println(F("no_concuerda_Encoder_pasos_X "));
    Serial.print(M1.cont_pasos_encoder_por_ciclo);
    Serial.print("|");
    Serial.print(Md1.encoder_debug);
    Serial.print("|");
  }
  if(M1.dir){
    Ep1.pul_posicion_eje_act+=M1.cont_pulsos_interface_por_ciclo;
  }else{
    Ep1.pul_posicion_eje_act-=M1.cont_pulsos_interface_por_ciclo;
  }
  if(Ep1.pul_posicion_eje_act>=Ep1.pul_limite_positivo_max){
    //desabilitar_movimiento_positivo
    //Activar_fin_de_carrera_soft
  }
  if(Ep1.pul_posicion_eje_act<=Ep1.pul_limite_negativo_max){
    //desabilitar_movimiento_negativo
    //Activar_fin_de_carrera_soft
  }
}
void actualizar_setpoint_X(){
  if(M1.pasos_encoder_inerciales>3){
    Serial.println(F("error sobre pulso encoder_X"));
    Serial.print(M1.pasos_encoder_inerciales);
    if(M1.pasos_encoder_inerciales>10){
      Serial.print(F("Requiere Calibrado_X"));
      Me1.error_pasos_encoder_perdidos+=M1.pasos_encoder_inerciales;
      M1.pasos_encoder_inerciales=0;
    }
  }
  if(M1.cont_pulsos_interfase_extra>3){
    Serial.println(F("error sobre pul interface_X"));
    Serial.print(M1.cont_pulsos_interfase_extra);
    if(M1.cont_pulsos_interfase_extra>10){
      Serial.print(F("Requiere Calibrado_X"));
      Me1.error_pulsos_interfase_extra+=M1.cont_pulsos_interfase_extra;
      M1.cont_pulsos_interfase_extra=0;
    }
  }
  if(M1.dir_act){
    M1.setpoint_posicion_act+=M1.pasos_encoder_inerciales;
    M1.setpoint_posicion_act-=M1.pasos_encoder_faltantes;
  }else{
    M1.setpoint_posicion_act-=M1.pasos_encoder_inerciales;
    M1.setpoint_posicion_act+=M1.pasos_encoder_faltantes;
  }
  M1.setpoint_posicion_act-=M1.cont_pulsos_interfase_extra;
  Me1.error_posicion_motor=M1.setpoint_posicion_act;
  if(abs(Me1.error_posicion_motor)>9){
    Me1.error_pasos_encoder_perdidos+=Me1.error_posicion_motor;
    Me1.error_posicion_motor=0;
    M1.setpoint_posicion_act=0;
    Serial.print(F("error mayor_X_encoder a 0.1: "));
    Serial.print(Me1.error_pasos_encoder_perdidos);
    Serial.println(F("Requiere Calibrado_X"));
  }
}
void inicializacion_variables_ciclo_X(){
  M1.setpoint_posicion_prev=M1.setpoint_posicion_act;
  M1.dir_act=M1.dir;
  M1.t_inicio_mov=millis();
  M1.t_pul=millis()-M1.t_pul_inicio;
  M1.t_pul_inicio=millis();
}
void encerado_variables_ciclo_X(){
  M1.enable_intento_correccion_posicion=0;
  M1.cont_pulsos_interfase_extra=0;
  M1.cont_pulsos_interface_por_ciclo=0;///////////////////////////*******debug_pul
  M1.cont_pasos_encoder_por_ciclo=0;////////////////////////*******debug_encoder
    Md1.encoder_debug=0;
    Md1.pul_interf_debug=0;
  M1.cont_pasos_encoder_correctos=0;
  M1.cont_pul=0;
  M1.encoder_act=0;
  M1.pasos_encoder_inerciales=0;
  M1.pasos_encoder_faltantes=0;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////MOTOR_X/////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////MOTOR_X/////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////MOTOR_X/////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////MOTOR_Y/////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////MOTOR_Y/////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////MOTOR_Y/////////////////////////////////////////////////////////////
void control_a_pasos_servomotor_eje_Y(){
  if(M2.est_motor==0){ // ServoMotor Parado
    if(M2.encoder_act>0){ // ServoMotor Inercia
      estado_motor_inercia_Y0();
    }
    if(M2.cont_pul==1){// ServoMotor Comenzar Movimiento
      estado_comenzar_movimiento_Y1();
    }else{
      if(M2.cont_pul>1){
        estado_mov_Y3();
        estado_comenzar_movimiento_Y1();
      }
    }
  }
  if(M2.est_motor==1){
    if(M2.cont_pul>0){// Pulso de Interfase Extra
      estado_mov_Y4();
    }
    M2.t_mov=millis()-M2.t_inicio_mov;
    aceleracion_motor_proporcional_Y();
    verificar_encoder_tiempo_en_movimiento_Y();
  }
  if(M2.est_motor==2){
    M2.t_inercia=millis()-M2.t_fin_mov;
    frenado_motor_Y();
  }
}
void estado_motor_inercia_Y0(){
  Md2.t2=millis()-M2.t_inicio_mov;
  //M2.pasos_encoder_inerciales+=M2.encoder_act;
  if(M2.pasos_encoder_faltantes>0){
    M2.pasos_encoder_faltantes-=M2.encoder_act;
    M2.cont_pasos_encoder_correctos+=M2.encoder_act;
  }else{
    M2.pasos_encoder_inerciales+=M2.encoder_act;
  }
  M2.cont_pasos_encoder_por_ciclo+=M2.encoder_act;
  M2.encoder_act=0;
}
void estado_comenzar_movimiento_Y1(){
  //verificacion_motor_Y();
  //////////////////////////////////////////////////////////////////*********************************************
  if(M2.deshabilita_movP_motor || M2.deshabilita_movN_motor){
    Me2.error_pulsos_interfase_extra+=M2.cont_pulsos_interfase_extra;
    encerado_variables_ciclo_Y();
    M2.setpoint_posicion_act=0;
  }
  //////////////////////////////////////////////////////////////////*********************************************
  actualizar_posicion_eje_Y();
  M2.pasos_encoder_inerciales+=M2.encoder_act;
  actualizar_setpoint_Y();
       imprimir_datos_ciclo_inercia_Y();
  inicializacion_variables_ciclo_Y();
  encerado_variables_ciclo_Y();
  validacion_movimiento_encoder_Y();
  /////////////////////////////////////////////////////////////////////
  if(M2.enable_movimiento_motor){
    if(M2.variable_velocidad_movimiento_motor==1){ //Acelera proporcionalmente
      M2.enable_frenar_al_finalizar=1;
      M2.enable_aceleracion_proporcional=1;
      M2.est_motor=1;
      M2.tiempo_encendido_motor=15; // desde 4-15 [ms] milisegundos
      //M2.tiempo_encendido_motor=tiempo_aceleracion_normal; // desde 4-15 [ms] milisegundos
      mover_motor_Y(M2.dir_act, 255);
    }
    if(M2.variable_velocidad_movimiento_motor==2){
      M2.est_motor=1;      
      M2.tiempo_encendido_motor=15;
      //M2.tiempo_encendido_motor=map(M2.t_pul,4,15,9,7);
      mover_motor_Y(M2.dir_act, 255);
    }
  }else{
    M2.variable_velocidad_movimiento_motor=0;
    M2.tiempo_encendido_motor=0;
    //estado_mov_Y5();
    digitalWrite(M2.PIN_A,true);
    digitalWrite(M2.PIN_B,true);
    M2.t_fin_mov=millis();
    Md2.t1=millis()-M2.t_inicio_mov;
    M2.enable_aceleracion_proporcional=0;
    M2.cont_pasos_encoder_en_movimiento=0;
    //M2.dir_prev=M2.dir_act;
    M2.est_motor=2;
    imprimir_datos_ciclo_Y();
  }
}
void estado_mov_Y3(){
  if(M2.dir){
    M2.cont_pulsos_interfase_extra+=M2.cont_pul;
    M2.cont_pulsos_interfase_extra-=1;
  }else{
    M2.cont_pulsos_interfase_extra-=M2.cont_pul;
    M2.cont_pulsos_interfase_extra+=1;
  }
}
void estado_mov_Y4(){
  if(M2.dir==M2.dir_act){
    if(M2.enable_aceleracion_proporcional){
      M2.enable_aceleracion_proporcional=0;
      M2.enable_frenar_al_finalizar=0;
      aceleracion_motor_instantanea_Y();
    }
    if(M2.cont_pul>1){
      Serial.print("ERROR_ demasiados_PULSOS_Y");
    }
    M2.tiempo_encendido_motor=14;
    M2.cont_pulsos_interface_por_ciclo+=M2.cont_pul;////////////////////////*******debug_pul
    M2.cont_pul=0;
    M2.setpoint_encoder+=1;
    if(M2.setpoint_encoder>2){
      M2.setpoint_encoder=2;
      if(M2.dir){
        M2.cont_pulsos_interfase_extra+=1;
      }else{
        M2.cont_pulsos_interfase_extra-=1;
      }
    }
  }else{
    if(M2.dir){
      M2.cont_pulsos_interfase_extra+=1;
    }else{
      M2.cont_pulsos_interfase_extra-=1;
    }
  }
}
void aceleracion_motor_proporcional_Y(){
  if(M2.enable_aceleracion_proporcional){
    if(M2.t_mov!=M2.variable_cont_aceleracion){
      analogWrite(M2.PIN_VELOCIDAD,255-(M2.t_mov*(variable_aceleracion_corto/tiempo_aceleracion_corto)));
      M2.variable_cont_aceleracion=M2.t_mov;
    }
  }
}

void aceleracion_motor_instantanea_Y(){
  mover_motor_Y(M2.dir_act, 255);
}
void verificar_encoder_tiempo_en_movimiento_Y(){
  if(M2.enable_movimiento_motor){
    if(M2.encoder_act==M2.setpoint_encoder){
      estado_mov_Y5();
    }else{
      if(M2.t_mov>M2.tiempo_encendido_motor){
        if(M2.encoder_act<M2.setpoint_encoder){ // encoder falta de moverse
          M2.pasos_encoder_faltantes=M2.setpoint_encoder-M2.encoder_act;
        }
        if(M2.encoder_act>M2.setpoint_encoder){ // encoder se movio de mas
          M2.pasos_encoder_inerciales+=M2.encoder_act-M2.setpoint_encoder;
        }
        estado_mov_Y5();
      }else{
        if(M2.encoder_act>M2.setpoint_encoder){
          M2.pasos_encoder_inerciales+=M2.encoder_act-M2.setpoint_encoder;
          estado_mov_Y5();
        }
      }
    }
  }else{
    estado_mov_Y5();
  }
}

void estado_mov_Y5(){
  digitalWrite(M2.PIN_A,true);
  digitalWrite(M2.PIN_B,true);
  M2.cont_pasos_encoder_correctos=M2.encoder_act;
  M2.t_fin_mov=millis();
  Md2.t1=millis()-M2.t_inicio_mov;
  M2.enable_aceleracion_proporcional=0;
  M2.cont_pasos_encoder_en_movimiento=M2.encoder_act;// con ese puedo saber si recibi o fue por tiempo
  M2.cont_pasos_encoder_por_ciclo+=M2.encoder_act;////////////////////////*******debug_encoder
  M2.encoder_act=0;
  //M2.dir_prev=M2.dir_act;
  M2.est_motor=2;
  imprimir_datos_ciclo_Y();
}
void frenado_motor_Y(){
  if(M2.enable_frenar_al_finalizar){
    if(M2.t_inercia==1){
      analogWrite(M2.PIN_VELOCIDAD,255);
      if(!M2.dir_act){
        digitalWrite(M2.PIN_A,true);
        digitalWrite(M2.PIN_B,false);
      }else{
        digitalWrite(M2.PIN_A,false);
        digitalWrite(M2.PIN_B,true);
      }
    }
    if(M2.t_inercia>=2){
      digitalWrite(M2.PIN_A,true);
      digitalWrite(M2.PIN_B,true);
      M2.enable_frenar_al_finalizar=0;
      M2.est_motor=0;
    }
  }else{
    M2.est_motor=0;
  }
}
void validacion_movimiento_encoder_Y(){
  if((M2.dir && M2.deshabilita_movP_motor) || (!M2.dir && M2.deshabilita_movN_motor)){
      M2.setpoint_encoder=0;      
      M2.enable_movimiento_motor=0;
  }else{

      
    if(M2.setpoint_posicion_act==0){
      M2.setpoint_encoder=1;      
      M2.enable_movimiento_motor=1;
    }else{
      if(M2.dir_act){
        if(M2.setpoint_posicion_act>0){
          M2.setpoint_posicion_act-=1;
          M2.enable_movimiento_motor=0;
          M2.setpoint_encoder=0;
        }else{
          if(M2.setpoint_posicion_act<0){
            M2.setpoint_encoder=2;
            M2.setpoint_posicion_act+=1;
            M2.enable_intento_correccion_posicion=1;///////////////////////*/*/*/*
          }else{
            M2.setpoint_encoder=1;
          }
          M2.enable_movimiento_motor=1;
        }
      }else{
        if(M2.setpoint_posicion_act<0){
          M2.setpoint_posicion_act+=1;
          M2.enable_movimiento_motor=0;
          M2.setpoint_encoder=0;
        }else{
          if(M2.setpoint_posicion_act>0){
            M2.setpoint_encoder=2;
            M2.setpoint_posicion_act-=1;
            M2.enable_intento_correccion_posicion=1;///////////////////////*/*/*/*
          }else{
            M2.setpoint_encoder=1;
          }
          M2.enable_movimiento_motor=1;
        }
      }
    }
    if(M2.enable_movimiento_motor){
      if(M2.t_pul>15 && M2.setpoint_encoder==1){
        M2.variable_velocidad_movimiento_motor=1;
      }else{
        M2.variable_velocidad_movimiento_motor=2;
      }
    }else{
      M2.variable_velocidad_movimiento_motor=0;
    }
  
  }
}
/////////////////////////////////////////////////==INTERRUPCIONES==//////////////////////
void inter_encoder_Y1(){
  M2.encoder_act +=1;
  Md2.encoder_debug+=1;
}

void inter_pulsos_Y1(){
  M2.dir=digitalRead(M2.PIN_DIR);
  M2.cont_pul += 1;
  Md2.pul_interf_debug+=1;
}
//////////////////////////////////////////////////***PRINT****///////////////////////////
void imprimir_datos_ciclo_Y(){
  if(DEBUG_Y){
    Serial.println("");
    cont_Y++;
    if(cont_Y>9){
      Serial.print(cont_Y);
    }else{
      Serial.print("0");
      Serial.print(cont_Y);
    }
    if(cont_Y>95){
      cont_Y=0;
    }
    if(M2.dir_act){
      Serial.print("|P");
    }else{
      Serial.print("|N");
    }
    if(M2.t_pul>99){
      Serial.print("99"); // Tiempo entre pulsos
    }else{
      if(M2.t_pul>9){
        Serial.print(M2.t_pul); // Tiempo entre pulsos
      }else{
        Serial.print("0");
        Serial.print(M2.t_pul);
      }
    }
    Serial.print("|D");
    if(M2.tiempo_encendido_motor>9){
      Serial.print(M2.tiempo_encendido_motor);
    }else{
      Serial.print("0");
      Serial.print(M2.tiempo_encendido_motor);
    }
    Serial.print("|t1|");
    if(Md2.t1>9){
      Serial.print(Md2.t1);
    }else{
      Serial.print("0");
      Serial.print(Md2.t1);
    }
    Serial.print("|");
    Serial.print(M2.cont_pasos_encoder_en_movimiento); //Pulsos recibidos asta estadoV5
    Serial.print("|SP|");
    Serial.print(M2.setpoint_encoder);
    //Serial.print("|");
 }
}
void imprimir_datos_ciclo_inercia_Y(){
  if(DEBUG_Y){
//    Serial.print("|F|");
//    Serial.print(M2.pasos_encoder_faltantes); ////print_inercia
    Serial.print("|");
    Serial.print(M2.cont_pasos_encoder_correctos); ////print_inercia
    Serial.print("|I|");
    Serial.print(M2.pasos_encoder_inerciales);////print_inercia
//    Serial.print("|M|");
    Serial.print("|t2|");     //datos ciclo anterior
    if(M2.pasos_encoder_inerciales>0){
      if(Md2.t2>9){
        Serial.print(Md2.t2);
      }else{
        Serial.print("0");
        Serial.print(Md2.t2);
      }
    }else{
      Serial.print("  ");
    }
    //////////////////DEBUG//////////////////////
    //Serial.print("|");
    Serial.print("|");
    Serial.print(M2.enable_intento_correccion_posicion); ////print_inercia
    Serial.print("|P_Y|");
    Serial.print(M2.cont_pulsos_interfase_extra); //print_inercia
//    Serial.print("|C|");  
//    Serial.print(Md2.cont_pasos_encoder_totales);
//    Serial.print("|");
//    Serial.print(Md2.cont_pasos_encoder_totales-Md2.cont_pulsos_interface_totales);    
//    Serial.print("|p|");
//    Serial.print(Md2.cont_pulsos_interface_totales);
//    Serial.print("|");
    Serial.print("|L|");
    Serial.print(M2.cont_pasos_encoder_por_ciclo);////////////////////////*******debug_encoder //print_inercia
    Serial.print("|");
    Serial.print(M2.cont_pulsos_interface_por_ciclo);////////////////////////*******debug_pul //print_inercia
    Serial.print("|");
    Serial.print(M2.cont_pasos_encoder_por_ciclo-M2.cont_pulsos_interface_por_ciclo);////////////////////////*******debug_encoder
    Serial.print("|E|");
    Serial.print(Me2.error_posicion_motor);
    Serial.print("|S|");
    Serial.print(Ep2.pul_posicion_eje_act);
  }
}
//////////////////////////////////////////////////***PRINT****///////////////////////////

void verificacion_motor_Y(){
  if(Md2.encoder_debug==0){
    M2.cont_motor_desconectado++;
    if(M2.cont_motor_desconectado>2){
      Serial.println("MotorNoResponde_Y");
      //error_motor_no_responde=1;
      M2.cont_motor_desconectado=0;
      if(M2.dir_act){
        M2.deshabilita_movP_motor=1;
        Serial.print("P");
      }else{
        M2.deshabilita_movN_motor=1;
        Serial.print("N");
      }
      Serial.print("Requiere Calibrado_Y");
       //habilitar fin de carrera
       //enviar stop y pausa
       //Recomendar Calibrar
    }
  }else{
    M2.cont_motor_desconectado=0;
  }
}

void mover_motor_Y(bool direccion, uint8_t velocidad){
  analogWrite(M2.PIN_VELOCIDAD,velocidad);
  if(direccion){
    digitalWrite(M2.PIN_A,true);
    digitalWrite(M2.PIN_B,false);
  }else{
    digitalWrite(M2.PIN_A,false);
    digitalWrite(M2.PIN_B,true);
  }
}

void actualizar_posicion_eje_Y(){
  M2.cont_pulsos_interface_por_ciclo+=M2.cont_pul;////////////////////////*******debug_pul
  M2.cont_pasos_encoder_por_ciclo+=M2.encoder_act;////////////////////////*******debug_encoder
//  Md2.cont_pasos_encoder_totales+=Md2.encoder_debug;
//  Md2.cont_pulsos_interface_totales+=Md2.pul_interf_debug;
  if(M2.cont_pulsos_interface_por_ciclo!=Md2.pul_interf_debug){
    Serial.println(F("no_concuerda_pul_interfase_Y "));
    Serial.print(M2.cont_pulsos_interface_por_ciclo);
    Serial.print("|");
    Serial.print(Md2.pul_interf_debug);
    Serial.print("|");
  }
  if(M2.cont_pasos_encoder_por_ciclo!=Md2.encoder_debug){
    Serial.println(F("no_concuerda_Encoder_pasos_Y "));
    Serial.print(M2.cont_pasos_encoder_por_ciclo);
    Serial.print("|");
    Serial.print(Md2.encoder_debug);
    Serial.print("|");
  }
  if(M2.dir){
    Ep2.pul_posicion_eje_act+=M2.cont_pulsos_interface_por_ciclo;
  }else{
    Ep2.pul_posicion_eje_act-=M2.cont_pulsos_interface_por_ciclo;
  }
  if(Ep2.pul_posicion_eje_act>=Ep2.pul_limite_positivo_max){
    //desabilitar_movimiento_positivo
    //Activar_fin_de_carrera_soft
  }
  if(Ep2.pul_posicion_eje_act<=Ep2.pul_limite_negativo_max){
    //desabilitar_movimiento_negativo
    //Activar_fin_de_carrera_soft
  }
}
void actualizar_setpoint_Y(){
  if(M2.pasos_encoder_inerciales>3){
    Serial.println(F("error sobre pulso encoder_Y"));
    Serial.print(M2.pasos_encoder_inerciales);
    if(M2.pasos_encoder_inerciales>10){
      Serial.print(F("Requiere Calibrado_Y"));
      Me2.error_pasos_encoder_perdidos+=M2.pasos_encoder_inerciales;
      M2.pasos_encoder_inerciales=0;
    }
  }
  if(M2.cont_pulsos_interfase_extra>3){
    Serial.println(F("error sobre pul interface_Y"));
    Serial.print(M2.cont_pulsos_interfase_extra);
    if(M2.cont_pulsos_interfase_extra>10){
      Serial.print(F("Requiere Calibrado_Y"));
      Me2.error_pulsos_interfase_extra+=M2.cont_pulsos_interfase_extra;
      M2.cont_pulsos_interfase_extra=0;
    }
  }
  if(M2.dir_act){
    M2.setpoint_posicion_act+=M2.pasos_encoder_inerciales;
    M2.setpoint_posicion_act-=M2.pasos_encoder_faltantes;
  }else{
    M2.setpoint_posicion_act-=M2.pasos_encoder_inerciales;
    M2.setpoint_posicion_act+=M2.pasos_encoder_faltantes;
  }
  M2.setpoint_posicion_act-=M2.cont_pulsos_interfase_extra;
  Me2.error_posicion_motor=M2.setpoint_posicion_act;
  if(abs(Me2.error_posicion_motor)>9){
    Me2.error_pasos_encoder_perdidos+=Me2.error_posicion_motor;
    Me2.error_posicion_motor=0;
    M2.setpoint_posicion_act=0;
    Serial.println(F("error mayor_Y_encoder a 0.1: "));
    Serial.print(Me2.error_pasos_encoder_perdidos);
    Serial.print(F("Requiere Calibrado_Y"));
  }
}
void inicializacion_variables_ciclo_Y(){
  M2.setpoint_posicion_prev=M2.setpoint_posicion_act;
  M2.dir_act=M2.dir;
  M2.t_inicio_mov=millis();
  M2.t_pul=millis()-M2.t_pul_inicio;
  M2.t_pul_inicio=millis();
}
void encerado_variables_ciclo_Y(){
  M2.enable_intento_correccion_posicion=0;
  M2.cont_pulsos_interfase_extra=0;
  M2.cont_pulsos_interface_por_ciclo=0;///////////////////////////*******debug_pul
  M2.cont_pasos_encoder_por_ciclo=0;////////////////////////*******debug_encoder
    Md2.encoder_debug=0;
    Md2.pul_interf_debug=0;
  M2.cont_pasos_encoder_correctos=0;
  M2.cont_pul=0;
  M2.encoder_act=0;
  M2.pasos_encoder_inerciales=0;
  M2.pasos_encoder_faltantes=0;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////MOTOR_Z/////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////MOTOR_Z/////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////MOTOR_Z/////////////////////////////////////////////////////////////
void control_a_pasos_servomotor_eje_Z(){
  if(M3.est_motor==0){ // ServoMotor Parado
    if(M3.encoder_act>0){ // ServoMotor Inercia
      estado_motor_inercia_Z0();
    }
    if(M3.cont_pul==1){// ServoMotor Comenzar Movimiento
      estado_comenzar_movimiento_Z1();
    }else{
      if(M3.cont_pul>1){
        estado_mov_Z3();
        estado_comenzar_movimiento_Z1();
      }
    }
  }
  if(M3.est_motor==1){
    if(M3.cont_pul>0){// Pulso de Interfase Extra
      estado_mov_Z4();
    }
    M3.t_mov=millis()-M3.t_inicio_mov;
    aceleracion_motor_proporcional_Z();
    verificar_encoder_tiempo_en_movimiento_Z();
  }
  if(M3.est_motor==2){
    M3.t_inercia=millis()-M3.t_fin_mov;
    frenado_motor_Z();
  }
}
void estado_motor_inercia_Z0(){
  Md3.t2=millis()-M3.t_inicio_mov;
  //M3.pasos_encoder_inerciales+=M3.encoder_act;
  if(M3.pasos_encoder_faltantes>0){
    M3.pasos_encoder_faltantes-=M3.encoder_act;
    M3.cont_pasos_encoder_correctos+=M3.encoder_act;
  }else{
    M3.pasos_encoder_inerciales+=M3.encoder_act;
  }
  M3.cont_pasos_encoder_por_ciclo+=M3.encoder_act;
  M3.encoder_act=0;
}
void estado_comenzar_movimiento_Z1(){
  verificacion_motor_Z();
  //////////////////////////////////////////////////////////////////*********************************************
  if(M3.deshabilita_movP_motor || M3.deshabilita_movN_motor){
    Me3.error_pulsos_interfase_extra+=M3.cont_pulsos_interfase_extra;
    encerado_variables_ciclo_Z();
    M3.setpoint_posicion_act=0;
  }
  //////////////////////////////////////////////////////////////////*********************************************
  actualizar_posicion_eje_Z();
  M3.pasos_encoder_inerciales+=M3.encoder_act;
  actualizar_setpoint_Z();
       imprimir_datos_ciclo_inercia_Z();
  inicializacion_variables_ciclo_Z();
  encerado_variables_ciclo_Z();
  validacion_movimiento_encoder_Z();
  /////////////////////////////////////////////////////////////////////
  if(M3.enable_movimiento_motor){
    if(M3.variable_velocidad_movimiento_motor==1){ //Acelera proporcionalmente
      M3.enable_frenar_al_finalizar=1;
      M3.enable_aceleracion_proporcional=1;
      M3.est_motor=1;
      M3.tiempo_encendido_motor=15; // desde 4-15 [ms] milisegundos
      //M3.tiempo_encendido_motor=tiempo_aceleracion_normal; // desde 4-15 [ms] milisegundos
      mover_motor_Z(M3.dir_act, 255);
    }
    if(M3.variable_velocidad_movimiento_motor==2){
      M3.est_motor=1;      
      M3.tiempo_encendido_motor=15;
      //M3.tiempo_encendido_motor=map(M3.t_pul,4,15,9,7);
      mover_motor_Z(M3.dir_act, 255);
    }
  }else{
    M3.variable_velocidad_movimiento_motor=0;
    M3.tiempo_encendido_motor=0;
    //estado_mov_Z5();
    digitalWrite(M3.PIN_A,true);
    digitalWrite(M3.PIN_B,true);
    M3.t_fin_mov=millis();
    Md3.t1=millis()-M3.t_inicio_mov;
    M3.enable_aceleracion_proporcional=0;
    M3.cont_pasos_encoder_en_movimiento=0;
    //M3.dir_prev=M3.dir_act;
    M3.est_motor=2;
    imprimir_datos_ciclo_Z();
  }
}
void estado_mov_Z3(){
  if(M3.dir){
    M3.cont_pulsos_interfase_extra+=M3.cont_pul;
    M3.cont_pulsos_interfase_extra-=1;
  }else{
    M3.cont_pulsos_interfase_extra-=M3.cont_pul;
    M3.cont_pulsos_interfase_extra+=1;
  }
}
void estado_mov_Z4(){
  if(M3.dir==M3.dir_act){
    if(M3.enable_aceleracion_proporcional){
      M3.enable_aceleracion_proporcional=0;
      M3.enable_frenar_al_finalizar=0;
      aceleracion_motor_instantanea_Z();
    }
    if(M3.cont_pul>1){
      Serial.print("ERROR_ demasiados_PULSOS_Z");
    }
    M3.tiempo_encendido_motor=14;
    M3.cont_pulsos_interface_por_ciclo+=M3.cont_pul;////////////////////////*******debug_pul
    M3.cont_pul=0;
    M3.setpoint_encoder+=1;
    if(M3.setpoint_encoder>2){
      M3.setpoint_encoder=2;
      if(M3.dir){
        M3.cont_pulsos_interfase_extra+=1;
      }else{
        M3.cont_pulsos_interfase_extra-=1;
      }
    }
  }else{
    if(M3.dir){
      M3.cont_pulsos_interfase_extra+=1;
    }else{
      M3.cont_pulsos_interfase_extra-=1;
    }
  }
}
void aceleracion_motor_proporcional_Z(){
  if(M3.enable_aceleracion_proporcional){
    if(M3.t_mov!=M3.variable_cont_aceleracion){
      analogWrite(M3.PIN_VELOCIDAD,255-(M3.t_mov*(variable_aceleracion_corto/tiempo_aceleracion_corto)));
      M3.variable_cont_aceleracion=M3.t_mov;
    }
  }
}

void aceleracion_motor_instantanea_Z(){
  mover_motor_Z(M3.dir_act, 255);
}
void verificar_encoder_tiempo_en_movimiento_Z(){
  if(M3.enable_movimiento_motor){
    if(M3.encoder_act==M3.setpoint_encoder){
      estado_mov_Z5();
    }else{
      if(M3.t_mov>M3.tiempo_encendido_motor){
        if(M3.encoder_act<M3.setpoint_encoder){ // encoder falta de moverse
          M3.pasos_encoder_faltantes=M3.setpoint_encoder-M3.encoder_act;
        }
        if(M3.encoder_act>M3.setpoint_encoder){ // encoder se movio de mas
          M3.pasos_encoder_inerciales+=M3.encoder_act-M3.setpoint_encoder;
        }
        estado_mov_Z5();
      }else{
        if(M3.encoder_act>M3.setpoint_encoder){
          M3.pasos_encoder_inerciales+=M3.encoder_act-M3.setpoint_encoder;
          estado_mov_Z5();
        }
      }
    }
  }else{
    estado_mov_Z5();
  }
}

void estado_mov_Z5(){
  digitalWrite(M3.PIN_A,true);
  digitalWrite(M3.PIN_B,true);
  M3.cont_pasos_encoder_correctos=M3.encoder_act;
  M3.t_fin_mov=millis();
  Md3.t1=millis()-M3.t_inicio_mov;
  M3.enable_aceleracion_proporcional=0;
  M3.cont_pasos_encoder_en_movimiento=M3.encoder_act;// con ese puedo saber si recibi o fue por tiempo
  M3.cont_pasos_encoder_por_ciclo+=M3.encoder_act;////////////////////////*******debug_encoder
  M3.encoder_act=0;
  //M3.dir_prev=M3.dir_act;
  M3.est_motor=2;
  imprimir_datos_ciclo_Z();
}
void frenado_motor_Z(){
  if(M3.enable_frenar_al_finalizar){
    if(M3.t_inercia==1){
      analogWrite(M3.PIN_VELOCIDAD,255);
      if(!M3.dir_act){
        digitalWrite(M3.PIN_A,true);
        digitalWrite(M3.PIN_B,false);
      }else{
        digitalWrite(M3.PIN_A,false);
        digitalWrite(M3.PIN_B,true);
      }
    }
    if(M3.t_inercia>=2){
      digitalWrite(M3.PIN_A,true);
      digitalWrite(M3.PIN_B,true);
      M3.enable_frenar_al_finalizar=0;
      M3.est_motor=0;
    }
  }else{
    M3.est_motor=0;
  }
}
void validacion_movimiento_encoder_Z(){
  if((M3.dir && M3.deshabilita_movP_motor) || (!M3.dir && M3.deshabilita_movN_motor)){
      M3.setpoint_encoder=0;      
      M3.enable_movimiento_motor=0;
  }else{

      
    if(M3.setpoint_posicion_act==0){
      M3.setpoint_encoder=1;      
      M3.enable_movimiento_motor=1;
    }else{
      if(M3.dir_act){
        if(M3.setpoint_posicion_act>0){
          M3.setpoint_posicion_act-=1;
          M3.enable_movimiento_motor=0;
          M3.setpoint_encoder=0;
        }else{
          if(M3.setpoint_posicion_act<0){
            M3.setpoint_encoder=2;
            M3.setpoint_posicion_act+=1;
            M3.enable_intento_correccion_posicion=1;///////////////////////*/*/*/*
          }else{
            M3.setpoint_encoder=1;
          }
          M3.enable_movimiento_motor=1;
        }
      }else{
        if(M3.setpoint_posicion_act<0){
          M3.setpoint_posicion_act+=1;
          M3.enable_movimiento_motor=0;
          M3.setpoint_encoder=0;
        }else{
          if(M3.setpoint_posicion_act>0){
            M3.setpoint_encoder=2;
            M3.setpoint_posicion_act-=1;
            M3.enable_intento_correccion_posicion=1;///////////////////////*/*/*/*
          }else{
            M3.setpoint_encoder=1;
          }
          M3.enable_movimiento_motor=1;
        }
      }
    }
    if(M3.enable_movimiento_motor){
      if(M3.t_pul>15 && M3.setpoint_encoder==1){
        M3.variable_velocidad_movimiento_motor=1;
      }else{
        M3.variable_velocidad_movimiento_motor=2;
      }
    }else{
      M3.variable_velocidad_movimiento_motor=0;
    }
  
  }
}
/////////////////////////////////////////////////==INTERRUPCIONES==//////////////////////
void inter_encoder_Z1(){
  M3.encoder_act +=1;
  Md3.encoder_debug+=1;
}

void inter_pulsos_Z1(){
  M3.dir=digitalRead(M3.PIN_DIR);
  M3.cont_pul += 1;
  Md3.pul_interf_debug+=1;
}
//////////////////////////////////////////////////***PRINT****///////////////////////////
void imprimir_datos_ciclo_Z(){
  if(DEBUG_Z){
    Serial.println("");
    cont_Z++;
    if(cont_Z>9){
      Serial.print(cont_Z);
    }else{
      Serial.print("0");
      Serial.print(cont_Z);
    }
    if(cont_Z>95){
      cont_Z=0;
    }
    if(M3.dir_act){
      Serial.print("|P");
    }else{
      Serial.print("|N");
    }
    if(M3.t_pul>99){
      Serial.print("99"); // Tiempo entre pulsos
    }else{
      if(M3.t_pul>9){
        Serial.print(M3.t_pul); // Tiempo entre pulsos
      }else{
        Serial.print("0");
        Serial.print(M3.t_pul);
      }
    }
    Serial.print("|D");
    if(M3.tiempo_encendido_motor>9){
      Serial.print(M3.tiempo_encendido_motor);
    }else{
      Serial.print("0");
      Serial.print(M3.tiempo_encendido_motor);
    }
    Serial.print("|t1|");
    if(Md3.t1>9){
      Serial.print(Md3.t1);
    }else{
      Serial.print("0");
      Serial.print(Md3.t1);
    }
    Serial.print("|");
    Serial.print(M3.cont_pasos_encoder_en_movimiento); //Pulsos recibidos asta estadoV5
    Serial.print("|SP|");
    Serial.print(M3.setpoint_encoder);
    //Serial.print("|");
 }
}
void imprimir_datos_ciclo_inercia_Z(){
  if(DEBUG_Z){
//    Serial.print("|F|");
//    Serial.print(M3.pasos_encoder_faltantes); ////print_inercia
    Serial.print("|");
    Serial.print(M3.cont_pasos_encoder_correctos); ////print_inercia
    Serial.print("|I|");
    Serial.print(M3.pasos_encoder_inerciales);////print_inercia
//    Serial.print("|M|");
    Serial.print("|t2|");     //datos ciclo anterior
    if(M3.pasos_encoder_inerciales>0){
      if(Md3.t2>9){
        Serial.print(Md3.t2);
      }else{
        Serial.print("0");
        Serial.print(Md3.t2);
      }
    }else{
      Serial.print("  ");
    }
    //////////////////DEBUG//////////////////////
    //Serial.print("|");
    Serial.print("|");
    Serial.print(M3.enable_intento_correccion_posicion); ////print_inercia
    Serial.print("|P_Z|");
    Serial.print(M3.cont_pulsos_interfase_extra); //print_inercia
//    Serial.print("|C|");  
//    Serial.print(Md3.cont_pasos_encoder_totales);
//    Serial.print("|");
//    Serial.print(Md3.cont_pasos_encoder_totales-Md3.cont_pulsos_interface_totales);    
//    Serial.print("|p|");
//    Serial.print(Md3.cont_pulsos_interface_totales);
//    Serial.print("|");
    Serial.print("|L|");
    Serial.print(M3.cont_pasos_encoder_por_ciclo);////////////////////////*******debug_encoder //print_inercia
    Serial.print("|");
    Serial.print(M3.cont_pulsos_interface_por_ciclo);////////////////////////*******debug_pul //print_inercia
    Serial.print("|");
    Serial.print(M3.cont_pasos_encoder_por_ciclo-M3.cont_pulsos_interface_por_ciclo);////////////////////////*******debug_encoder
    Serial.print("|E|");
    Serial.print(Me3.error_posicion_motor);
    Serial.print("|S|");
    Serial.print(Ep3.pul_posicion_eje_act);
  }
}
//////////////////////////////////////////////////***PRINT****///////////////////////////

void verificacion_motor_Z(){
  if(Md3.encoder_debug==0){
    M3.cont_motor_desconectado++;
    if(M3.cont_motor_desconectado>2){
      Serial.println("MotorNoResponde_Z");
      //error_motor_no_responde=1;
      M3.cont_motor_desconectado=0;
      if(M3.dir_act){
        M3.deshabilita_movP_motor=1;
        Serial.print("P");
      }else{
        M3.deshabilita_movN_motor=1;
        Serial.print("N");
      }
      Serial.print("Requiere Calibrado_Z");
       //habilitar fin de carrera
       //enviar stop y pausa
       //Recomendar Calibrar
    }
  }else{
    M3.cont_motor_desconectado=0;
  }
}

void mover_motor_Z(bool direccion, uint8_t velocidad){
  analogWrite(M3.PIN_VELOCIDAD,velocidad);
  if(direccion){
    digitalWrite(M3.PIN_A,true);
    digitalWrite(M3.PIN_B,false);
  }else{
    digitalWrite(M3.PIN_A,false);
    digitalWrite(M3.PIN_B,true);
  }
}

void actualizar_posicion_eje_Z(){
  M3.cont_pulsos_interface_por_ciclo+=M3.cont_pul;////////////////////////*******debug_pul
  M3.cont_pasos_encoder_por_ciclo+=M3.encoder_act;////////////////////////*******debug_encoder
//  Md3.cont_pasos_encoder_totales+=Md3.encoder_debug;
//  Md3.cont_pulsos_interface_totales+=Md3.pul_interf_debug;
  if(M3.cont_pulsos_interface_por_ciclo!=Md3.pul_interf_debug){
    Serial.println(F("no_concuerda_pul_interfase_Z "));
    Serial.print(M3.cont_pulsos_interface_por_ciclo);
    Serial.print("|");
    Serial.print(Md3.pul_interf_debug);
    Serial.print("|");
  }
  if(M3.cont_pasos_encoder_por_ciclo!=Md3.encoder_debug){
    Serial.println(F("no_concuerda_Encoder_pasos_Z "));
    Serial.print(M3.cont_pasos_encoder_por_ciclo);
    Serial.print("|");
    Serial.print(Md3.encoder_debug);
    Serial.print("|");
  }
  if(M3.dir){
    Ep3.pul_posicion_eje_act+=M3.cont_pulsos_interface_por_ciclo;
  }else{
    Ep3.pul_posicion_eje_act-=M3.cont_pulsos_interface_por_ciclo;
  }
  if(Ep3.pul_posicion_eje_act>=Ep3.pul_limite_positivo_max){
    //desabilitar_movimiento_positivo
    //Activar_fin_de_carrera_soft
  }
  if(Ep3.pul_posicion_eje_act<=Ep3.pul_limite_negativo_max){
    //desabilitar_movimiento_negativo
    //Activar_fin_de_carrera_soft
  }
}
void actualizar_setpoint_Z(){
  if(M3.pasos_encoder_inerciales>3){
    Serial.println(F("error sobre pulso encoder_Z"));
    Serial.print(M3.pasos_encoder_inerciales);
    if(M3.pasos_encoder_inerciales>10){
      Serial.print(F("Requiere Calibrado_Z"));
      Me3.error_pasos_encoder_perdidos+=M3.pasos_encoder_inerciales;
      M3.pasos_encoder_inerciales=0;
    }
  }
  if(M3.cont_pulsos_interfase_extra>3){
    Serial.println(F("error sobre pul interface_Z"));
    Serial.print(M3.cont_pulsos_interfase_extra);
    if(M3.cont_pulsos_interfase_extra>10){
      Serial.print(F("Requiere Calibrado_Z"));
      Me3.error_pulsos_interfase_extra+=M3.cont_pulsos_interfase_extra;
      M3.cont_pulsos_interfase_extra=0;
    }
  }
  if(M3.dir_act){
    M3.setpoint_posicion_act+=M3.pasos_encoder_inerciales;
    M3.setpoint_posicion_act-=M3.pasos_encoder_faltantes;
  }else{
    M3.setpoint_posicion_act-=M3.pasos_encoder_inerciales;
    M3.setpoint_posicion_act+=M3.pasos_encoder_faltantes;
  }
  M3.setpoint_posicion_act-=M3.cont_pulsos_interfase_extra;
  Me3.error_posicion_motor=M3.setpoint_posicion_act;
  if(abs(Me3.error_posicion_motor)>9){
    Me3.error_pasos_encoder_perdidos+=Me3.error_posicion_motor;
    Me3.error_posicion_motor=0;
    M3.setpoint_posicion_act=0;
    Serial.println(F("error mayor_Z_encoder a 0.1: "));
    Serial.print(Me3.error_pasos_encoder_perdidos);
    Serial.print(F("Requiere Calibrado_Z"));
  }
}
void inicializacion_variables_ciclo_Z(){
  M3.setpoint_posicion_prev=M3.setpoint_posicion_act;
  M3.dir_act=M3.dir;
  M3.t_inicio_mov=millis();
  M3.t_pul=millis()-M3.t_pul_inicio;
  M3.t_pul_inicio=millis();
}
void encerado_variables_ciclo_Z(){
  M3.enable_intento_correccion_posicion=0;
  M3.cont_pulsos_interfase_extra=0;
  M3.cont_pulsos_interface_por_ciclo=0;///////////////////////////*******debug_pul
  M3.cont_pasos_encoder_por_ciclo=0;////////////////////////*******debug_encoder
    Md3.encoder_debug=0;
    Md3.pul_interf_debug=0;
  M3.cont_pasos_encoder_correctos=0;
  M3.cont_pul=0;
  M3.encoder_act=0;
  M3.pasos_encoder_inerciales=0;
  M3.pasos_encoder_faltantes=0;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////MOTOR_Z/////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////MOTOR_Z/////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////MOTOR_Z/////////////////////////////////////////////////////////////


////COMUNICACION_CP_y_MANDO////////COMUNICACION_CP_y_MANDO////////COMUNICACION_CP_y_MANDO////////COMUNICACION_CP_y_MANDO////////COMUNICACION_CP_y_MANDO////
////COMUNICACION_CP_y_MANDO////////COMUNICACION_CP_y_MANDO////////COMUNICACION_CP_y_MANDO////////COMUNICACION_CP_y_MANDO////////COMUNICACION_CP_y_MANDO////
////COMUNICACION_CP_y_MANDO////////COMUNICACION_CP_y_MANDO////////COMUNICACION_CP_y_MANDO////////COMUNICACION_CP_y_MANDO////////COMUNICACION_CP_y_MANDO////


void Recepcion_Dato_MANDO(){
  char c1 = Serial1.read();
  MANDO.rev=1;
  Buscar_comandos_MANDO(c1);
  if(MANDO.rev) grafcet_MANDO(c1);
}
void Buscar_comandos_MANDO(char c){ // Todos los comandos que puede enviar el MANDO
  if(MANDO.enviando_comando_tipo==0){
    MANDO.rev=0;
      switch (c) {
        case 'P': //peticion_recibida
          MANDO.enviando_comando_tipo=1;
          MANDO.EP1=1;
        break;
//        case 'T': //trabajo
//          MANDO.enviando_comando_tipo=2;
//          MANDO.ET1=1;
//        break;
//        case 'E': // estado_recibida
//          MANDO.enviando_comando_tipo=3;
//          MANDO.EE1=1;
//        break;
//        case 'R': //error
//          MANDO.enviando_comando_tipo=4;
//          MANDO.ER1=1;
//        break;
        case 'A': //accion_recibida
          MANDO.enviando_comando_tipo=5;
          MANDO.EA1=1;
        break;
        case 'S': //velocidad
          MANDO.enviando_comando_tipo=6;
          MANDO.ES1=1;
        break;
//        case 'V': //valvulas
//          MANDO.enviando_comando_tipo=7;
//          MANDO.EV1=1;
//        break;
//        case 'C': //cable
//          MANDO.enviando_comando_tipo=8;
//          MANDO.EC1=1;
//        break;
//        case 'M': //modo
//          MANDO.enviando_comando_tipo=9;
//          MANDO.EM1=1;
//        break;
        default:
          MANDO.rev=1;
          Serial.print(F("Error No se encuentra"));
          Serial.println(c);
        break;
      }
  }
}

void grafcet_MANDO(char c){
    switch (MANDO.enviando_comando_tipo) {
    case 1:
      grafcet_peticiones_MANDO(c); //depende con quien se trabaje
    break;
//    case 2:
//      grafcet_trabajos_MANDO(c); //recibe el trabajo actual
//    break;
//    case 3:
//      grafcet_estados_MANDO(c); //Recibe los estados de todos los modulos
//    break;
//    case 4:
//      grafcet_errores_MANDO(c);  //Recibe los estados de todos los modulos
//    break;
    case 5:
      grafcet_acciones_MANDO(c);  //depende con quien se trabaje
    break;
    case 6:
      grafcet_velocidad_MANDO(c); // solo funciona cuando se trabaja con NCL y NCM
    break;
//    case 7:
//      grafcet_valvulas_MANDO(c);
//    break;
//    case 8:
//      grafcet_cables_MANDO(c);
//    break;
//    case 9:
//      grafcet_modo_MANDO(c);
//    break;
    default:
    break;
  }
}

void grafcet_peticiones_MANDO(char c){
  if(MANDO.EP1){
    MANDO.EP1=0;
    MANDO.EP2=1;
    MANDO.rev=0;
    switch (c) {
      case 'E': //Estado_actual
        MANDO.peticion_recibida=1;
      break;
      case 'M': //Modo_actual
        MANDO.peticion_recibida=2;
      break;
      case 'A': //Alertas
        MANDO.peticion_recibida=3;
      break;
      case 'R': //Errores
        MANDO.peticion_recibida=4;
      break;
      case 'V': //Estado_Electrovalvulas
        MANDO.peticion_recibida=5;
      break;
//      case 'C': //Cables desconectados
//        MANDO.peticion_recibida=6;
//      break;      
      default:
        MANDO.rev=1;
        MANDO.EP1=0;
        MANDO.EP2=0;
        MANDO.enviando_comando_tipo=0;
        Serial.print(F("Error_P: "));
        Serial.println(c);
      break;
    }
  }
  if(MANDO.EP2 && MANDO.rev){
    MANDO.EP2=0;
    MANDO.enviando_comando_tipo=0;
    if(c==10 || c==13){
      MANDO.enable_enviado_peticion_completo=1;
      MANDO.recibido_nuevo_comando=1;
      Serial.print(F("MANDO.peticion_recibida: "));
      Serial.println(MANDO.peticion_recibida);
      MANDO.rev=0;
    }
  }
}


void grafcet_acciones_MANDO(char c){
  if(MANDO.EA1){
    MANDO.EA1=0;
    MANDO.EA2=1;
    MANDO.rev=0;
    switch (c) {
        case '1': //Spindler
          MANDO.accion_recibida=1;
        break;
        case '2': //Cooler
          MANDO.accion_recibida=2;
        break;
        case '3': //Piston de Agarre
          MANDO.accion_recibida=3;
        break;
        case '4': //Puerta
          MANDO.accion_recibida=4;
        break;
        case 'C': //Cambiar Modo Manual/Automatico
          MANDO.accion_recibida=5;
        break;
        //////////////////////////////////
//        case 'R': //Robot ir a posicion #
//          MANDO.accion_recibida=10;
//        break;
//        case 'I': //Riel Modo Manual o automatico
//          MANDO.accion_recibida=11;
//        break;
//        case 'J': //Riel ir a posicion #
//          MANDO.accion_recibida=12;
//        break;
        case 'M': // Mover manualmente eje # de NCM
          MANDO.accion_recibida=13;
        break;
        case 'L': // Mover manualmente eje # de NCL
          MANDO.accion_recibida=14;
        break;            
        default:
          MANDO.rev=1;
          MANDO.EA0=1;
          MANDO.EA2=0;
          MANDO.enviando_comando_tipo=0;
          Serial.print("Error_A: ");
          Serial.println(c);
        break;
    }
  }
  if(MANDO.EA2 && MANDO.rev){
    MANDO.EA3=1;
    MANDO.EA2=0;
    MANDO.rev=0;
    if(isdigit(c)){
      if(MANDO.accion_recibida<10){
        switch (c) {
          case '0':
            MANDO.accion_num_recibida=0;
          break;
          case '1':
            MANDO.accion_num_recibida=1;
          break;
          default:
            MANDO.rev=1;
            MANDO.EA3=0;
            MANDO.EA0=1;
            MANDO.accion_recibida=0;
            MANDO.enviando_comando_tipo=0;
            Serial.print("Error_Discreto: ");
            Serial.println(c);
          break;
        }
      }else{
        MANDO.accion_num_recibida=String(c).toInt();
      }
    }else{
      MANDO.rev=1;
      MANDO.EA3=0;
      MANDO.EA0=1;
      MANDO.accion_recibida=0;
      MANDO.enviando_comando_tipo=0;
      Serial.print("Error_subA: ");
      Serial.println(c);
    }
  }
  if(MANDO.EA3 && MANDO.rev){
    MANDO.EA3=0;
    MANDO.EA0=1;
    MANDO.enviando_comando_tipo=0;
    if(c==10 || c==13){
      MANDO.enviado_accion_completo=1;
      MANDO.recibido_nuevo_comando=1;
      Serial.print("MANDO.accion_recibida: ");
      Serial.print(MANDO.accion_recibida);
      Serial.print(" y ");
      Serial.println(MANDO.accion_num_recibida);
      MANDO.rev=0;
    }else{
      Serial.print("falto_enter: ");
      Serial.println(c);
    }
  }
}


void grafcet_velocidad_MANDO(char c){
  if(MANDO.ES1){
    MANDO.ES1=0;
    MANDO.ES2=1;
    MANDO.rev=0;
    if(isdigit(c)){
      MANDO.velocidad_recibida=String(c).toInt()*10;
    }else{
      MANDO.rev=1;
      MANDO.ES0=1;
      MANDO.ES2=0;
      MANDO.enviando_comando_tipo=0;
      MANDO.velocidad_recibida=0;
      Serial.print("Error_Speed1: ");
      Serial.println(c);
    }
  }
  if(MANDO.ES2 && MANDO.rev){
    MANDO.ES2=0;
    MANDO.ES3=1;
    MANDO.rev=0;
    if(isdigit(c)){
      MANDO.velocidad_recibida+=String(c).toInt();
    }else{
      MANDO.rev=1;
      MANDO.ES0=1;
      MANDO.ES3=0;
      MANDO.enviando_comando_tipo=0;
      MANDO.velocidad_recibida=0;
      Serial.print("Error_Speed2: ");
      Serial.println(c);
    }
  }
  if(MANDO.ES3 && MANDO.rev){
    MANDO.ES3=0;
    MANDO.ES0=1;
    MANDO.enviando_comando_tipo=0;
    if(c==10 || c==13){
      MANDO.enviado_velocidad_completo=1;
      MANDO.recibido_nuevo_comando=1;
      Serial.print("MANDO.velocidad_recibida: ");
      Serial.println(MANDO.velocidad_recibida);
      MANDO.rev=0;
    }else{
      Serial.print("falto_enter: ");
      Serial.println(c);
    }
  }
}


void Respuesta_a_comandos_MANDO(){
        if(MANDO.enable_enviado_peticion_completo){// Respuesta a las peticiones del MANDO
          MANDO.enable_enviado_peticion_completo=0;
          switch (MANDO.peticion_recibida) {
            case 1: //Estado_actual
                enviar_a_CP("E"+String(estado_act, DEC));
            break;
            case 2: //Modo_actual
                enviar_a_CP("M"+String(modo_act, DEC));
            break;
            case 3: //Alertas
                enviar_a_CP("R"+String(error_act, DEC));
            break;
            case 4: //Errores
                enviar_a_CP("R"+String(error_act, DEC));
            break;
            case 5: //Estado_Electrovalvulas
                enviar_a_CP("V"+String(valvulas_act, DEC)); 
            break;
      //      case 6: //Cables desconectados
      //      break;      
            default:
            break;
          }
          MANDO.peticion_recibida=0;
        }

//        if(estado_act==trabajando){
//             if(MANDO.enviado_accion_completo){
//                MANDO.enviado_accion_completo=0;
//                switch (MANDO.accion_recibida) {
//                  case 1: //Si Spindle se desactiva cuando se trabaja
//                      if(MANDO.accion_num_recibida==0){
//                        enable_off_spindle_cuando_se_trabaja=1;
//                      }else{
//                        MANDO.enviado_accion_completo=1;
//                      }
//                  break;
//                  case 3: //Puerta  
//                      if(MANDO.accion_num_recibida==0){
//                        enable_off_puerta_cuando_se_trabaja=1;
//                      }else{
//                        MANDO.enviado_accion_completo=1;
//                      }
//                  break;
//                  case 4: //Piston de Agarre
//                      if(MANDO.accion_num_recibida==0){
//                        enable_off_piston_agarre_cuando_se_trabaja=1;
//                      }else{
//                        MANDO.enviado_accion_completo=1;
//                      }
//                  break;
//                  case 13: // Mover manualmente eje # de NCM
//                      if(MANDO.accion_num_recibida>0){
//                        enable_off_ejes_cuando_se_trabaja=1;
//                      }else{
//                        MANDO.enviado_accion_completo=1;
//                      }
//                  break;
//                  case 14: // Mover manualmente eje # de NCL
//                      if(MANDO.accion_num_recibida>0){
//                        enable_off_ejes_cuando_se_trabaja=1;
//                      }else{
//                        MANDO.enviado_accion_completo=1;
//                      }
//                  break;                       
//                  default:
//                      MANDO.enviado_accion_completo=1;
//                  break;
//                }
//             }
//        }
        
            if(MANDO.enviado_accion_completo){
               MANDO.enviado_accion_completo=0;
                  switch (MANDO.accion_recibida) {
                    case 1: //Spindler
                      if(MANDO.accion_num_recibida==1){
                          digitalWrite(VAR_FREC_OUT,0);
                          valvulas_act|=0b00000001; //Añade un 1
                      }else{
                          digitalWrite(VAR_FREC_OUT,1);
                          valvulas_act&=0b11111110; //Añade un 0
                      }
                    break;
                    case 2: //Cooler
                      if(MANDO.accion_num_recibida==1){
                          digitalWrite(COOLER_OUT,0);
                          valvulas_act|=0b00000010; //Añade un 1
                      }else{
                          digitalWrite(COOLER_OUT,1);
                          valvulas_act&=0b11111101; //Añade un 0
                      }
                    break;
                    case 3: //Puerta  
                      if(MANDO.accion_num_recibida==1){
                          digitalWrite(PISTON_P_OUT,0);
                          valvulas_act|=0b00000100; //Añade un 1
                      }else{
                          digitalWrite(PISTON_P_OUT,1);
                          valvulas_act&=0b11111011; //Añade un 0
                      }
                    break;
                    case 4: //Piston de Agarre
                      if(MANDO.accion_num_recibida==1){
                          digitalWrite(PISTON_A_OUT,0);
                          valvulas_act|=0b00001000; //Añade un 1
                      }else{
                          digitalWrite(PISTON_A_OUT,1);
                          valvulas_act&=0b11110111; //Añade un 0
                      }
                    break;
                    case 5: //Cambiar Modo Manual/Automatico
                      if(MANDO.accion_num_recibida==1){
                          enable_cambiar_modo=1;
                          cambiar_a_modo=1;
                      }else{
                          enable_cambiar_modo=1;
                          cambiar_a_modo=0;
                      }
                    break;
                    //////////////////////////////////
                    case 13: // Mover manualmente eje # de NCM
                        if(MANDO.accion_num_recibida<4){
                            enable_mover_ejes_manualmente=1;
                            ejes_para_mover_manualmente=MANDO.accion_num_recibida;
                            //Serial.println("GRACET_rESPUESTA A COMANDO_FUNCIONA");
                        }else{
                          Serial.println("Eje no Existe NCM");
                        }
                    break;
                    case 14: // Mover manualmente eje # de NCL
                        if(MANDO.accion_num_recibida<3){
                            enable_mover_ejes_manualmente=1;
                            ejes_para_mover_manualmente=MANDO.accion_num_recibida;
                        }else{
                          Serial.println("Eje no Existe NCL");
                        }
                    break;            
                    default:
                        Serial.print("Actualizar_comunicacion_CP/accion: ");
                        Serial.println(MANDO.accion_recibida);
                    break;
                  }
              MANDO.accion_recibida=0;
              MANDO.accion_num_recibida=0;
            }
        if(MANDO.enviado_velocidad_completo){
          MANDO.enviado_velocidad_completo=0;
            velocidad_MANDO_act = map(MANDO.velocidad_recibida, 0, 99, 120, 255);
            Serial.print(velocidad_MANDO_act);
            enable_cambiar_velocidad=1;
          MANDO.velocidad_recibida=0;
        }
}

void Control_Principal(){

//  if(enable_cambiar_modo){
//    enable_cambiar_modo=0;
//    if(cambiar_a_modo){// manejar manualmente
//      modo_act=1;
//      digitalWrite(VAR_FREC_OUT,1); // Se para Spindle por precaucion
//      valvulas_act&=0b11111110; //Añade un 0
//      digitalWrite(COOLER_OUT,1);   // Se para Cooler por precaucion
//      valvulas_act&=0b11111101; //Añade un 0
//    }else{              // volver a modo normal
//      modo_act=0;
//    }
//    //Reset_Variables_cuando_cambio_modo();
//      M1.encoder_act=0;
//      Md1.encoder_debug=0;
//      M2.encoder_act=0;
//      Md2.encoder_debug=0;
//      M3.encoder_act=0;
//      Md3.encoder_debug=0;
//    
//      M1.cont_pul = 0;
//      Md1.pul_interf_debug=0;
//      M2.cont_pul = 0;
//      Md2.pul_interf_debug=0;
//      M3.cont_pul = 0;
//      Md3.pul_interf_debug=0;
//  }
//  if(modo_act==0){
//    if(digitalRead(SPINDLE_IN)){
//        digitalWrite(VAR_FREC_OUT,0);
//        valvulas_act|=0b00000001; //Añade un 1
//    }else{
//        digitalWrite(VAR_FREC_OUT,1);
//        valvulas_act&=0b11111110; //Añade un 0
//    }
//    if(digitalRead(COOLER_IN)){
//        digitalWrite(COOLER_OUT,0);
//        valvulas_act|=0b00000010; //Añade un 1
//    }else{
//        digitalWrite(COOLER_OUT,1);
//        valvulas_act&=0b11111101; //Añade un 0
//    }
//  }

  
  if(M1.est_motor==0 && M2.est_motor==0 && M3.est_motor==0){
      if(enable_mover_ejes_manualmente){
        //Serial.println("ENABLE_MANUAL");
          enable_mover_ejes_manualmente=0;
          digitalWrite(M1.PIN_A,true);
          digitalWrite(M1.PIN_B,true);
          digitalWrite(M2.PIN_A,true);
          digitalWrite(M2.PIN_B,true);
          digitalWrite(M3.PIN_A,true);
          digitalWrite(M3.PIN_B,true);
          encerado_variables_ciclo_X();
          encerado_variables_ciclo_Y();
          encerado_variables_ciclo_Z();
          encerar_deshabilitacion_motores();
        if(ejes_para_mover_manualmente==0){
          enable_habilitar_movimiento_desde_mando=0;
        }else{
          enable_habilitar_movimiento_desde_mando=1;
          enable_obligar_desactivacion_motores=1;
        }
      }

      if(!digitalRead(PUL_IN_EXT) && enable_habilitar_movimiento_desde_mando){
        //Serial.println("MOVER");
        ejes_para_mover_manualmente=MANDO.accion_num_recibida;//////////////////////////*******************************************************************
        enable_pul_activado=1;  
      }
      if(enable_pul_activado){
        if(E_velocidad0){
          if(millis()-t_manual_motor_velocidad>99-velocidad_MANDO_act){
            E_velocidad0=0;
            E_velocidad1=1;
            t_manual_motor_velocidad=millis();
            if(ejes_para_mover_manualmente==1){
              mover_motor_X(!digitalRead(DIR_IN_EXT), 255);
            }
            if(ejes_para_mover_manualmente==2){
              mover_motor_Y(!digitalRead(DIR_IN_EXT), 255);
            }
            if(ejes_para_mover_manualmente==3){
              mover_motor_Z(!digitalRead(DIR_IN_EXT), 255);
            }
          }
        }
        if(E_velocidad1){
          if(millis()-t_manual_motor_velocidad>velocidad_MANDO_act){
            E_velocidad1=0;
            E_velocidad0=1;
            t_manual_motor_velocidad=millis();
            digitalWrite(M1.PIN_A,true);
            digitalWrite(M1.PIN_B,true);
            digitalWrite(M2.PIN_A,true);
            digitalWrite(M2.PIN_B,true);
            digitalWrite(M3.PIN_A,true);
            digitalWrite(M3.PIN_B,true);
            encerado_variables_ciclo_X();
            encerado_variables_ciclo_Y();
            encerado_variables_ciclo_Z();
          }
        }
      }
  }
      if(digitalRead(PUL_IN_EXT) && enable_habilitar_movimiento_desde_mando){
        //Serial.print("STOP");}
        enable_pul_activado=0;
        E_velocidad0=0;
        E_velocidad1=0;
        digitalWrite(M1.PIN_A,true);
        digitalWrite(M1.PIN_B,true);
        digitalWrite(M2.PIN_A,true);
        digitalWrite(M2.PIN_B,true);
        digitalWrite(M3.PIN_A,true);
        digitalWrite(M3.PIN_B,true);
        encerado_variables_ciclo_X();
        encerado_variables_ciclo_Y();
        encerado_variables_ciclo_Z();
      }
      if(enable_obligar_desactivacion_motores && !enable_habilitar_movimiento_desde_mando){
        //Serial.println("OBLIGAR_STOP");
        enable_pul_activado=0;
        E_velocidad0=0;
        E_velocidad1=0;
        enable_obligar_desactivacion_motores=0;
        digitalWrite(M1.PIN_A,true);
        digitalWrite(M1.PIN_B,true);
        digitalWrite(M2.PIN_A,true);
        digitalWrite(M2.PIN_B,true);
        digitalWrite(M3.PIN_A,true);
        digitalWrite(M3.PIN_B,true);
        delay(500);
        encerado_variables_ciclo_X();
        encerado_variables_ciclo_Y();
        encerado_variables_ciclo_Z();
        encerar_deshabilitacion_motores();
      }
}

void encerar_deshabilitacion_motores(){
        M1.setpoint_posicion_act=0;
        M2.setpoint_posicion_act=0;
        M3.setpoint_posicion_act=0;
        M1.cont_motor_desconectado=0;
        M2.cont_motor_desconectado=0;
        M3.cont_motor_desconectado=0;
        M1.deshabilita_movP_motor=0;
        M1.deshabilita_movN_motor=0;
        M2.deshabilita_movP_motor=0;
        M2.deshabilita_movN_motor=0;
        M3.deshabilita_movP_motor=0;
        M3.deshabilita_movN_motor=0;
} 

////COMUNICACION_CP_y_MANDO////////COMUNICACION_CP_y_MANDO////////COMUNICACION_CP_y_MANDO////////COMUNICACION_CP_y_MANDO////////COMUNICACION_CP_y_MANDO////
////COMUNICACION_CP_y_MANDO////////COMUNICACION_CP_y_MANDO////////COMUNICACION_CP_y_MANDO////////COMUNICACION_CP_y_MANDO////////COMUNICACION_CP_y_MANDO////
////COMUNICACION_CP_y_MANDO////////COMUNICACION_CP_y_MANDO////////COMUNICACION_CP_y_MANDO////////COMUNICACION_CP_y_MANDO////////COMUNICACION_CP_y_MANDO////

