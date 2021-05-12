#include "mbed.h"
#include "mbed_rpc.h"
#include "MQTTNetwork.h"
#include "MQTTmbed.h"
#include "MQTTClient.h"
#include "stm32l475e_iot01_accelero.h"
#include "accelerometer_handler.h"
#include "config.h"
#include "magic_wand_model_data.h"

#include "tensorflow/lite/c/common.h"
#include "tensorflow/lite/micro/kernels/micro_ops.h"
#include "tensorflow/lite/micro/micro_error_reporter.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "tensorflow/lite/version.h"
#include "math.h"
#include "string.h"
#include "uLCD_4DGL.h"
using namespace std::chrono;

// For run a RPC loop with two custom functions (operation modes): 
// (1) gesture UI, and (2) tilt angle detection
BufferedSerial pc(USBTX, USBRX);
void a(Arguments *in, Reply *out);
void b(Arguments *in, Reply *out);
RPCFunction rpcGUI(&a, "a");
RPCFunction rpcAD(&b, "b");

typedef struct sNode {
  int seq_num;
  struct sNode *next;
} node;

typedef struct sSequence {
  int num;
  node *sequence;
} Sequence;

Sequence sArray[10];
int ArrayIndex = 0;

// LED
DigitalOut led1(LED1);
DigitalOut led2(LED2);
DigitalOut led3(LED3);

// uLCD
uLCD_4DGL uLCD(D1, D0, D2); // serial tx, serial rx, reset pin;

// GLOBAL VARIABLES
Timer debounce;                  //define debounce timer
WiFiInterface *wifi;
InterruptIn button(USER_BUTTON); //Interrupt on digital pushbutton input SW2
//InterruptIn btn3(SW3);
MQTT::Client<MQTTNetwork, Countdown>* rpcClient;
volatile int message_num = 0;
volatile int arrivedcount = 0;
volatile bool closed = false;


// The gesture index of the prediction
int gesture_index;
int sequence_num;

// For measuring the tilt angle;
int16_t rDataXYZ[3] = {0};
int16_t ppDataXYZ[3] = {0};
int idR[64] = {0};
int indexR = 0;

const char* topic = "Mbed";

Thread mqtt_thread(osPriorityNormal);
Thread thread(osPriorityHigh);
Thread rpcthread(osPriorityNormal);
//Thread t;
EventQueue mqtt_queue(64 * EVENTS_EVENT_SIZE);
EventQueue queue(64 * EVENTS_EVENT_SIZE);
EventQueue rpcqueue;

bool _confirm = false;  // whether user bottom is pressed
bool off1 = false;      // whether gesture UI mode is done
bool off2 = false;      // whether tilt angle mode is done
int select_angle = 0;    // the selected angle
int tiltcount = 0;      // the nuber of tilt events

void confirm_info() {
   // Note that printf is deferred with a call in the queue
   // It is not executed in the interrupt context
   printf("Confirm is triggered! \r\n");
}

void messageArrived(MQTT::MessageData& md) {
    MQTT::Message &message = md.message;
    char msg[300];
    //sprintf(msg, "Message arrived: QoS%d, retained %d, dup %d, packetID %d\r\n", message.qos, message.retained, message.dup, message.id);
    //printf(msg);
    //ThisThread::sleep_for(100ms);
    char payload[300];

    sprintf(payload, "Message arrived: %.*s\r\n", message.payloadlen, (char*)message.payload);
    printf("%s", payload);
    if (strcmp((char*)message.payload, "GestureUI Confirm") == 0) {
      off1 = true;
    } else if (strcmp((char*)message.payload, "Over tilt angle") == 0) {
      tiltcount++;
      if (tiltcount >= 10) off2 = true;
    }
    ++arrivedcount;
}

void publish_message(MQTT::Client<MQTTNetwork, Countdown>* client) {
    message_num++;
    MQTT::Message message;
    char buff[100];
    sprintf(buff, "GestureUI Confirm %d", message_num);
    message.qos = MQTT::QOS0;
    message.retained = false;
    message.dup = false;
    message.payload = (void*) buff;
    message.payloadlen = strlen(buff) + 1;
    int rc = client->publish(topic, message);

    //printf("rc:  %d\r\n", rc);
    printf("Puslish message: %s\r\n", buff);
}

void publish_message2(MQTT::Client<MQTTNetwork, Countdown>* client) {
    //message_num++;
    MQTT::Message message;
    char buff[100];
    sprintf(buff, "Over tilt angle");
    message.qos = MQTT::QOS0;
    message.retained = false;
    message.dup = false;
    message.payload = (void*) buff;
    message.payloadlen = strlen(buff) + 1;
    int rc = client->publish(topic, message);

    //printf("rc:  %d\r\n", rc);
    printf("Puslish message: %s\r\n", buff);
}

void Confirm()
{
  _confirm = true;
  //mqtt_queue.call(confirm_info);
}

void close_mqtt(void) {
    closed = true;
}

// Create an area of memory to use for input, output, and intermediate arrays.
// The size of this will depend on the model you're using, and may need to be
// determined by experimentation.
constexpr int kTensorArenaSize = 60 * 1024;
uint8_t tensor_arena[kTensorArenaSize];

// Return the result of the last prediction
int PredictGesture(float* ACCOutput) {
  // How many times the most recent gesture has been matched in a row
  static int continuous_count = 0;
  // The result of the last prediction
  static int last_predict = -1;

  // Find whichever output has a probability > 0.8 (they sum to 1)
  int this_predict = -1;
  for (int i = 0; i < label_num; i++) {
    if (ACCOutput[i] > 0.8) this_predict = i;
  }

  // No gesture was detected above the threshold
  if (this_predict == -1) {
    continuous_count = 0;
    last_predict = label_num;
    return label_num;
  }

  if (last_predict == this_predict) {
    continuous_count += 1;
  } else {
    continuous_count = 0;
  }
  last_predict = this_predict;

  // If we haven't yet had enough consecutive matches for this gesture,
  // report a negative result
  if (continuous_count < config.consecutiveInferenceThresholds[this_predict]) {
    return label_num;
  }
  // Otherwise, we've seen a positive result, so clear all our variables
  // and report it
  continuous_count = 0;
  last_predict = -1;

  return this_predict;
}

void RPCcall(void) {
  // receive commands, and send back the responses
  char buf[256], outbuf[256];

  FILE *devin = fdopen(&pc, "r");
  FILE *devout = fdopen(&pc, "w");

  while(1) {
    printf("Type command\n");
    memset(buf, 0, 256);
      for (int i = 0; ; i++) {
        char recv = fgetc(devin);
          if (recv == '\n') {
              printf("\r\n");
              break;
          }
          buf[i] = fputc(recv, devout);
      }
      //Call the static call method on the RPC class
      RPC::call(buf, outbuf);
    }
}

int main() {

  for (int i = 0; i < 10; i++) {
    sArray[i].num = 0;
    sArray[i].sequence = NULL;
  }

    wifi = WiFiInterface::get_default_instance();
    if (!wifi) {
            printf("ERROR: No WiFiInterface found.\r\n");
            return -1;
    }

    printf("\nConnecting to %s...\r\n", MBED_CONF_APP_WIFI_SSID);
    int ret = wifi->connect(MBED_CONF_APP_WIFI_SSID, MBED_CONF_APP_WIFI_PASSWORD, NSAPI_SECURITY_WPA_WPA2);
    if (ret != 0) {
            printf("\nConnection error: %d\r\n", ret);
            return -1;
    }

    NetworkInterface* net = wifi;
    MQTTNetwork mqttNetwork(net);
    MQTT::Client<MQTTNetwork, Countdown> client(mqttNetwork);
    rpcClient = &client;

    //TODO: revise host to your IP
    const char* host = "192.168.43.123";
    printf("Connecting to TCP network...\r\n");

    SocketAddress sockAddr;
    sockAddr.set_ip_address(host);
    sockAddr.set_port(1883);

    printf("address is %s/%d\r\n", (sockAddr.get_ip_address() ? sockAddr.get_ip_address() : "None"),  (sockAddr.get_port() ? sockAddr.get_port() : 0) ); //check setting

    int rc = mqttNetwork.connect(sockAddr);//(host, 1883);
    if (rc != 0) {
            printf("Connection error.");
            return -1;
    }
    printf("Successfully connected!\r\n");

    MQTTPacket_connectData data = MQTTPacket_connectData_initializer;
    data.MQTTVersion = 3;
    data.clientID.cstring = "Mbed";

    if ((rc = client.connect(data)) != 0){
            printf("Fail to connect MQTT\r\n");
    }
    if (client.subscribe(topic, MQTT::QOS0, messageArrived) != 0){
            printf("Fail to subscribe\r\n");
    }

    /*int num = 0;
    while (num != 5) {
            client.yield(100);
            ++num;
    }*/

    mqtt_thread.start(callback(&mqtt_queue, &EventQueue::dispatch_forever));
    thread.start(callback(&queue, &EventQueue::dispatch_forever));
    rpcthread.start(callback(&rpcqueue, &EventQueue::dispatch_forever));

    BSP_ACCELERO_Init();
    rpcqueue.call(RPCcall);
    
    while (1) {
            if (closed) break;
            client.yield(100);
            ThisThread::sleep_for(100ms);
    }

    printf("Ready to close MQTT Network......\n");

    if ((rc = client.unsubscribe(topic)) != 0) {
            printf("Failed: rc from unsubscribe was %d\n", rc);
    }
    if ((rc = client.disconnect()) != 0) {
    printf("Failed: rc from disconnect was %d\n", rc);
    }

    mqttNetwork.disconnect();
    printf("Successfully closed!\n");

    return 0;
}

void Feature_info() {
      uLCD.cls();
      uLCD.printf("\n Classified gesture is:\n");
      if (gesture_index == 0) {
        uLCD.printf("\nGesture ID: RING:\n\r");
        uLCD.printf("\nSequence number of the event: %d\n", sequence_num);
      } else if (gesture_index == 1) {      // 45 degree
        uLCD.printf("\nGesture ID: SLOPE\n\r");
        uLCD.printf("\nSequence number of the event: %d\n", sequence_num);
      } else if (gesture_index == 2) {      // 60 degree
        uLCD.printf("\nGesture ID: DOWN:\n\r");
        uLCD.printf("\nSequence number of the event: %d\n", sequence_num);
      }
}

void Freq_confirm() {
      uLCD.printf("\n Confirm!\n");
}

void nowAngle(double value) {
      uLCD.cls();
      uLCD.printf("\n Now angle is:\n");
      uLCD.printf("\n %g\n", value);
}

node *newNode(bool s) {
  node *n;

  n = new(node);
  n->seq_num = (int)s;
  n->next = NULL;

  return n;
}

bool detect(float *ACC1, float *ACC2) {
  double angle = 0;
  double value = 0;
  value = (ACC1[0] * ACC2[0] + ACC1[1] * ACC2[1] + ACC1[2] * ACC2[2]);
  value = value / sqrt(ACC1[0] * ACC1[0] + ACC1[1] * ACC1[1] + ACC1[2] * ACC1[2]);
  value = value / sqrt(ACC2[0] * ACC2[0] + ACC2[1] * ACC2[1] + ACC2[2] * ACC2[2]);
  angle = acos(value);
  angle = angle / 3.14 * 180;
  if (angle > 15.0) return true;
  else return false;
}

void selectAngle() {
    // Whether we should clear the buffer next time we fetch data
  bool should_clear_buffer = false;
  bool got_data = false;
  bool this_num = false;
  float lastACC[3] = {0, 0, 0};
  node *tmp;
  node *ttmp;

  // Set up logging.
  static tflite::MicroErrorReporter micro_error_reporter;
  tflite::ErrorReporter* error_reporter = &micro_error_reporter;

  // Map the model into a usable data structure. This doesn't involve any
  // copying or parsing, it's a very lightweight operation.
  const tflite::Model* model = tflite::GetModel(g_magic_wand_model_data);
  if (model->version() != TFLITE_SCHEMA_VERSION) {
    error_reporter->Report(
        "Model provided is schema version %d not equal "
        "to supported version %d.",
        model->version(), TFLITE_SCHEMA_VERSION);
    return;
  }

  // Pull in only the operation implementations we need.
  // This relies on a complete list of all the ops needed by this graph.
  // An easier approach is to just use the AllOpsResolver, but this will
  // incur some penalty in code space for op implementations that are not
  // needed by this graph.
  static tflite::MicroOpResolver<6> micro_op_resolver;
  micro_op_resolver.AddBuiltin(
      tflite::BuiltinOperator_DEPTHWISE_CONV_2D,
      tflite::ops::micro::Register_DEPTHWISE_CONV_2D());
  micro_op_resolver.AddBuiltin(tflite::BuiltinOperator_MAX_POOL_2D,
                               tflite::ops::micro::Register_MAX_POOL_2D());
  micro_op_resolver.AddBuiltin(tflite::BuiltinOperator_CONV_2D,
                               tflite::ops::micro::Register_CONV_2D());
  micro_op_resolver.AddBuiltin(tflite::BuiltinOperator_FULLY_CONNECTED,
                               tflite::ops::micro::Register_FULLY_CONNECTED());
  micro_op_resolver.AddBuiltin(tflite::BuiltinOperator_SOFTMAX,
                               tflite::ops::micro::Register_SOFTMAX());
  micro_op_resolver.AddBuiltin(tflite::BuiltinOperator_RESHAPE,
                               tflite::ops::micro::Register_RESHAPE(), 1);

  // Build an interpreter to run the model with
  static tflite::MicroInterpreter static_interpreter(
      model, micro_op_resolver, tensor_arena, kTensorArenaSize, error_reporter);
  tflite::MicroInterpreter* interpreter = &static_interpreter;

  // Allocate memory from the tensor_arena for the model's tensors
  interpreter->AllocateTensors();

  // Obtain pointer to the model's input tensor
  TfLiteTensor* model_input = interpreter->input(0);
  if ((model_input->dims->size != 4) || (model_input->dims->data[0] != 1) ||
      (model_input->dims->data[1] != config.seq_length) ||
      (model_input->dims->data[2] != kChannelNumber) ||
      (model_input->type != kTfLiteFloat32)) {
    error_reporter->Report("Bad input tensor parameters in model");
    return;
  }

  int input_length = model_input->bytes / sizeof(float);

  TfLiteStatus setup_status = SetupAccelerometer(error_reporter);
  if (setup_status != kTfLiteOk) {
    error_reporter->Report("Set up failed\n");
    return;
  }

  //error_reporter->Report("Set up successful...\n");
  ArrayIndex = 0;
  tmp = sArray[ArrayIndex].sequence;
  while (true) {
    // Attempt to read new data from the accelerometer
    got_data = ReadAccelerometer(error_reporter, model_input->data.f,
                                 input_length, should_clear_buffer);

    // If there was no new data,
    // don't try to clear the buffer again and wait until next time
    if (!got_data) {
      should_clear_buffer = false;
      continue;
    }

    // Run inference, and report any error
    TfLiteStatus invoke_status = interpreter->Invoke();
    if (invoke_status != kTfLiteOk) {
      error_reporter->Report("Invoke failed on index: %d\n", begin_index);
      continue;
    }

    // Analyze the results to obtain a prediction
    gesture_index = PredictGesture(interpreter->output(0)->data.f);
    this_num = detect(lastACC, interpreter->output(0)->data.f);
    for (int k = 0; k < 3; k++) lastACC[k] = interpreter->output(0)->data.f[k];
    tmp = newNode(this_num);
    ttmp = sArray[ArrayIndex].sequence;
    //while (ttmp->next != NULL) ttmp = ttmp->next;
    //ttmp->next = tmp;
    printf("%d\n", tmp->seq_num);
    sArray[ArrayIndex].num++;

    // Clear the buffer next time we read data
    should_clear_buffer = gesture_index < label_num;

    //mqtt_thread.start(callback(&mqtt_queue, &EventQueue::dispatch_forever));
    button.rise(&Confirm);

    // Produce an output
    if (gesture_index < label_num) {
      error_reporter->Report(config.output_message[gesture_index]);
      if (gesture_index == 0) select_angle = 30;
      else if (gesture_index == 1) select_angle = 45;
      else if (gesture_index == 2) select_angle = 60;
      queue.call(Feature_info);
      ArrayIndex++;
      //queue.call(&publish_message, rpcClient);
    }
  }
}

void a(Arguments *in, Reply *out) {
  sequence_num = 0;
  printf("Gesture UI Mode On!\n");
  // start a thread function
  mqtt_queue.call(selectAngle);
  while (!off1) {
    led1 = !led1;
    ThisThread::sleep_for(200ms);
  }
  led1 = 0;
  off1 = false;
}

void record(void) {
  double angle = 0;
  double absr = 0, abspp = 0;
  double value = 0;

  BSP_ACCELERO_AccGetXYZ(ppDataXYZ);
  led3 = !led3;
  value = (rDataXYZ[0] * ppDataXYZ[0] + rDataXYZ[1] * ppDataXYZ[1] + rDataXYZ[2] * ppDataXYZ[2]);
  value = value / sqrt(rDataXYZ[0] * rDataXYZ[0] + rDataXYZ[1] * rDataXYZ[1] + rDataXYZ[2] * rDataXYZ[2]);
  value = value / sqrt(ppDataXYZ[0] * ppDataXYZ[0] + ppDataXYZ[1] * ppDataXYZ[1] + ppDataXYZ[2] * ppDataXYZ[2]);
  angle = acos(value);
  angle = angle / 3.14 * 180;
  uLCD.cls();
  uLCD.printf("\n Now angle is:\n");
  uLCD.printf("\n %g\n", angle);
  //queue.call(&nowAngle, angle);

  if (angle > select_angle) {
    queue.call(&publish_message2, rpcClient);
  }
}

void stopRecord(void) {
    printf("---TiltAngle Stop---\n");
    for (auto &i : idR)
      queue.cancel(i);
}

void b(Arguments *in, Reply *out) {
  printf("Tilt angle detection mode on!\n");
  printf("Reference Measurement Start!\n");
  _confirm = false;
  off2 = false;
  tiltcount = 0;
  button.rise(&Confirm);
  while (!_confirm) {
    led1 = !led1;
    led2 = !led2;
    BSP_ACCELERO_AccGetXYZ(rDataXYZ);
    ThisThread::sleep_for(200ms);
  }
  led1 = led2 = 0;
  ThisThread::sleep_for(1000ms);
  printf("---TiltAngle Start---\n");
  while (!off2) {
    idR[indexR++] = queue.call(record);
    indexR = indexR % 64;
    ThisThread::sleep_for(200ms);
  }
  for (auto &i : idR) queue.cancel(i);
  ThisThread::sleep_for(2000ms);
  printf("---TiltAngle Stop---\n");
  _confirm = false;
  off2 = false;
  led3 = 0;
}
