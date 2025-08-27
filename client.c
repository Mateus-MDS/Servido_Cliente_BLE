 #include <stdio.h>             // Biblioteca padrão para entrada/saída (printf, etc.)
 #include "btstack.h"           // Biblioteca padrão para funcionalidades do Raspberry Pi Pico (GPIO, timers, etc.)
 #include "pico/cyw43_arch.h"   // Biblioteca específica para a arquitetura CYW43 (Wi-Fi/Bluetooth no Pico W)
 #include "pico/stdlib.h"       // Biblioteca BTstack para funcionalidades Bluetooth

 #include "hardware/i2c.h"

 // Bibliotecas espec�ficas do projeto
#include "ssd1306.h"      // Driver do display OLED
#include "font.h"         // Fontes para o display

// Configura��o I2C para Display OLED
#define I2C_PORT_DISP i2c1            // I2C1 para display
#define I2C_SDA_DISP 14               // Pino SDA do display
#define I2C_SCL_DISP 15               // Pino SCL do display
#define ENDERECO_DISP 0x3C            // Endere�o I2C do display SSD1306

float temp;
 
 #if 0
 #define DEBUG_LOG(...) printf(__VA_ARGS__) // Define macro para logs de depuração (desativado por padrão)
 #else
 #define DEBUG_LOG(...)
 #endif
 
 #define LED_QUICK_FLASH_DELAY_MS 100       // Atraso em ms para piscar rápido no LED
 #define LED_SLOW_FLASH_DELAY_MS 1000       // Atraso em ms para piscar lento no LED
 
 typedef enum {
     TC_OFF,                                // Cliente desligado
     TC_IDLE,                               // Cliente ocioso
     TC_W4_SCAN_RESULT,                     // Aguardando resultado de varredura BLE
     TC_W4_CONNECT,                         // Aguardando conexão com dispositivo
     TC_W4_SERVICE_RESULT,                  // Aguardando descoberta de serviço
     TC_W4_CHARACTERISTIC_RESULT,           // Aguardando descoberta de característica
     TC_W4_ENABLE_NOTIFICATIONS_COMPLETE,   // Aguardando habilitação de notificações
     TC_W4_READY                            // Cliente pronto para processar dados
 } gc_state_t;
 
 // Callback para eventos HCI
 static btstack_packet_callback_registration_t hci_event_callback_registration;
 static gc_state_t state = TC_OFF;
 static bd_addr_t server_addr;
 static bd_addr_type_t server_addr_type;
 static hci_con_handle_t connection_handle;
 static gatt_client_service_t server_service;
 static gatt_client_characteristic_t server_characteristic;
 // protocolo GATT (Generic Attribute Profile)
 static bool listener_registered;
 static gatt_client_notification_t notification_listener;
 static btstack_timer_source_t heartbeat;
 
// Protótipo de funções
static void client_start(void);
static bool advertisement_report_contains_service(uint16_t service, uint8_t *advertisement_report);
static void handle_gatt_client_event(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);
static void hci_event_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);
static void heartbeat_handler(struct btstack_timer_source *ts);

int main() {

    // Inicializa o sistema de entrada e saída padrão (printf, etc.)
     stdio_init_all();
 
    // Inicializa a arquitetura CYW43 (Wi-Fi e Bluetooth)
     if (cyw43_arch_init()) {
        // Caso a inicialização falhe, imprime mensagem de erro e encerra o programa
         printf("Falha para inicializar o periférico cyw43_arch\n");
         return -1;
     }
 
    // Inicializa o protocolo L2CAP (canal lógico no Bluetooth)
     l2cap_init();

     // Inicializa o Security Manager (SM) para BLE
     sm_init();

     // Define as capacidades de entrada/saída como "sem entrada ou saída" (usado para emparelhamento BLE)
     sm_set_io_capabilities(IO_CAPABILITY_NO_INPUT_NO_OUTPUT);
 
     // Configura o servidor ATT (Attribute Protocol), mas sem callbacks (NULL)
     att_server_init(NULL, NULL, NULL);
 
     // Inicializa o cliente GATT para interagir com dispositivos BLE
     gatt_client_init();
     
     // Registra o callback para eventos HCI, associando a função hci_event_handler
     hci_event_callback_registration.callback = &hci_event_handler;
     hci_add_event_handler(&hci_event_callback_registration);
 
     // Configura um timer para controlar o LED e associa ao handler heartbeat_handler
     heartbeat.process = &heartbeat_handler;
     btstack_run_loop_set_timer(&heartbeat, LED_SLOW_FLASH_DELAY_MS);
     btstack_run_loop_add_timer(&heartbeat);
 
     // Liga o Bluetooth no controlador (modo ON)
     hci_power_control(HCI_POWER_ON);

     // Inicializa I2C1 para o display OLED em 400kHz
     i2c_init(I2C_PORT_DISP, 400 * 1000);
     gpio_set_function(I2C_SDA_DISP, GPIO_FUNC_I2C);
     gpio_set_function(I2C_SCL_DISP, GPIO_FUNC_I2C);
     gpio_pull_up(I2C_SDA_DISP);
     gpio_pull_up(I2C_SCL_DISP);

     // Configura��o e inicializa��o do display SSD1306
     ssd1306_t ssd;
     ssd1306_init(&ssd, WIDTH, HEIGHT, false, ENDERECO_DISP, I2C_PORT_DISP);
     ssd1306_config(&ssd);
     ssd1306_send_data(&ssd);

     // Limpa o display inicialmente
     ssd1306_fill(&ssd, false);
     ssd1306_send_data(&ssd);

     bool cor = true;
 
     // btstack_run_loop_execute is only required when using the 'polling' method (e.g. using pico_cyw43_arch_poll library).
     // This example uses the 'threadsafe background` method, where BT work is handled in a low priority IRQ, so it
     // is fine to call bt_stack_run_loop_execute() but equally you can continue executing user code.
 
 #if 0 // this is only necessary when using polling (which we aren't, but we're showing it is still safe to call in this case)
     btstack_run_loop_execute();
 #else
     // this core is free to do it's own stuff except when using 'polling' method (in which case you should use 
     // btstacK_run_loop_ methods to add work to the run loop.
 
     // this is a forever loop in place of where user code would go.
     while(true) {    

     ssd1306_fill(&ssd, !cor);  // Limpa o display
        
     if (connection_handle == HCI_CON_HANDLE_INVALID) {
        // Mostrar no display "Desconectado"
        ssd1306_draw_string(&ssd, "BLE Estado: ", 20, 41);
        ssd1306_draw_string(&ssd, "Desconectado", 15, 52);
     } else {
        // Mostrar no display "Conectado"
        ssd1306_draw_string(&ssd, "BLE Estado: ", 20, 41);
        ssd1306_draw_string(&ssd, "Conectado", 20, 52);
     }

     printf("Conectado a %s\n", bd_addr_to_str(server_addr));

     char Temp_string[20];   // espa�o suficiente

     snprintf(Temp_string, sizeof(Temp_string), "Temp: %.2f C", temp/100);

     // Desenha interface de captura
            ssd1306_rect(&ssd, 3, 3, 122, 60, cor, !cor);
            ssd1306_line(&ssd, 3, 25, 123, 25, cor);
            ssd1306_line(&ssd, 3, 37, 123, 37, cor);
            ssd1306_draw_string(&ssd, "CLIENTE", 33, 6);
            ssd1306_draw_string(&ssd, "BLUETOOTH", 25, 16);
            ssd1306_draw_string(&ssd, Temp_string, 15, 28);
    
    ssd1306_send_data(&ssd);

         sleep_ms(700);
     }
 #endif
     return 0;
 }

 //------------------------------ Funções ---------------------------------------

 static void client_start(void){
    DEBUG_LOG("Start scanning!\n");
    state = TC_W4_SCAN_RESULT;
    gap_set_scan_parameters(0,0x0030, 0x0030);
    gap_start_scan();
}

static bool advertisement_report_contains_service(uint16_t service, uint8_t *advertisement_report){
    // get advertisement from report event
    const uint8_t * adv_data = gap_event_advertising_report_get_data(advertisement_report);
    uint8_t adv_len  = gap_event_advertising_report_get_data_length(advertisement_report);

    // iterate over advertisement data
    ad_context_t context;
    for (ad_iterator_init(&context, adv_len, adv_data) ; ad_iterator_has_more(&context) ; ad_iterator_next(&context)){
        uint8_t data_type = ad_iterator_get_data_type(&context);
        uint8_t data_size = ad_iterator_get_data_len(&context);
        const uint8_t * data = ad_iterator_get_data(&context);
        switch (data_type){
            case BLUETOOTH_DATA_TYPE_COMPLETE_LIST_OF_16_BIT_SERVICE_CLASS_UUIDS:
                for (int i = 0; i < data_size; i += 2) {
                    uint16_t type = little_endian_read_16(data, i);
                    if (type == service) return true;
                }
            default:
                break;
        }
    }
    return false;
}

static void handle_gatt_client_event(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size) {
    UNUSED(packet_type);
    UNUSED(channel);
    UNUSED(size);

    uint8_t att_status;
    switch(state){
        case TC_W4_SERVICE_RESULT:
            switch(hci_event_packet_get_type(packet)) {
                case GATT_EVENT_SERVICE_QUERY_RESULT:
                    // store service (we expect only one)
                    DEBUG_LOG("Storing service\n");
                    gatt_event_service_query_result_get_service(packet, &server_service);
                    break;
                case GATT_EVENT_QUERY_COMPLETE:
                    att_status = gatt_event_query_complete_get_att_status(packet);
                    if (att_status != ATT_ERROR_SUCCESS){
                        printf("SERVICE_QUERY_RESULT, ATT Error 0x%02x.\n", att_status);
                        gap_disconnect(connection_handle);
                        break;  
                    } 
                    // service query complete, look for characteristic
                    state = TC_W4_CHARACTERISTIC_RESULT;
                    DEBUG_LOG("Search for env sensing characteristic.\n");
                    gatt_client_discover_characteristics_for_service_by_uuid16(handle_gatt_client_event, connection_handle, &server_service, ORG_BLUETOOTH_CHARACTERISTIC_TEMPERATURE);
                    break;
                default:
                    break;
            }
            break;
        case TC_W4_CHARACTERISTIC_RESULT:
            switch(hci_event_packet_get_type(packet)) {
                case GATT_EVENT_CHARACTERISTIC_QUERY_RESULT:
                    DEBUG_LOG("Storing characteristic\n");
                    gatt_event_characteristic_query_result_get_characteristic(packet, &server_characteristic);
                    break;
                case GATT_EVENT_QUERY_COMPLETE:
                    att_status = gatt_event_query_complete_get_att_status(packet);
                    if (att_status != ATT_ERROR_SUCCESS){
                        printf("CHARACTERISTIC_QUERY_RESULT, ATT Error 0x%02x.\n", att_status);
                        gap_disconnect(connection_handle);
                        break;  
                    } 
                    // register handler for notifications
                    listener_registered = true;
                    gatt_client_listen_for_characteristic_value_updates(&notification_listener, handle_gatt_client_event, connection_handle, &server_characteristic);
                    // enable notifications
                    DEBUG_LOG("Enable notify on characteristic.\n");
                    state = TC_W4_ENABLE_NOTIFICATIONS_COMPLETE;
                    gatt_client_write_client_characteristic_configuration(handle_gatt_client_event, connection_handle,
                        &server_characteristic, GATT_CLIENT_CHARACTERISTICS_CONFIGURATION_NOTIFICATION);
                    break;
                default:
                    break;
            }
            break;
        case TC_W4_ENABLE_NOTIFICATIONS_COMPLETE:
            switch(hci_event_packet_get_type(packet)) {
                case GATT_EVENT_QUERY_COMPLETE:
                    DEBUG_LOG("Notifications enabled, ATT status 0x%02x\n", gatt_event_query_complete_get_att_status(packet));
                    if (gatt_event_query_complete_get_att_status(packet) != ATT_ERROR_SUCCESS) break;
                    state = TC_W4_READY;
                    break;
                default:
                    break;
            }
            break;
        case TC_W4_READY:
            switch(hci_event_packet_get_type(packet)) {
                case GATT_EVENT_NOTIFICATION: {
                    uint16_t value_length = gatt_event_notification_get_value_length(packet);
                    const uint8_t *value = gatt_event_notification_get_value(packet);
                    DEBUG_LOG("Indication value len %d\n", value_length);
                    if (value_length == 2) {
                        temp = little_endian_read_16(value, 0);
                        printf("read temp %.2f degc\n", temp / 100);
                    } else {
                        printf("Unexpected length %d\n", value_length);
                    }
                    break;
                }
                default:
                    printf("Unknown packet type 0x%02x\n", hci_event_packet_get_type(packet));
                    break;
            }
            break;
        default:
            printf("error\n");
            break;
    }
}

static void hci_event_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size) {
    UNUSED(size);
    UNUSED(channel);
    bd_addr_t local_addr;
    if (packet_type != HCI_EVENT_PACKET) return;

    uint8_t event_type = hci_event_packet_get_type(packet);
    switch(event_type){
        case BTSTACK_EVENT_STATE:
            if (btstack_event_state_get_state(packet) == HCI_STATE_WORKING) {
                gap_local_bd_addr(local_addr);
                printf("BTstack up and running on %s.\n", bd_addr_to_str(local_addr));
                client_start();
            } else {
                state = TC_OFF;
            }
            break;
        case GAP_EVENT_ADVERTISING_REPORT:
            if (state != TC_W4_SCAN_RESULT) return;
            // check name in advertisement
            if (!advertisement_report_contains_service(ORG_BLUETOOTH_SERVICE_ENVIRONMENTAL_SENSING, packet)) return;
            // store address and type
            gap_event_advertising_report_get_address(packet, server_addr);
            server_addr_type = gap_event_advertising_report_get_address_type(packet);
            // stop scanning, and connect to the device
            state = TC_W4_CONNECT;
            gap_stop_scan();
            printf("Connecting to device with addr %s.\n", bd_addr_to_str(server_addr));
            gap_connect(server_addr, server_addr_type);
            break;
        case HCI_EVENT_LE_META:
            // wait for connection complete
            switch (hci_event_le_meta_get_subevent_code(packet)) {
                case HCI_SUBEVENT_LE_CONNECTION_COMPLETE:
                    if (state != TC_W4_CONNECT) return;
                    connection_handle = hci_subevent_le_connection_complete_get_connection_handle(packet);
                    // initialize gatt client context with handle, and add it to the list of active clients
                    // query primary services
                    DEBUG_LOG("Search for env sensing service.\n");
                    state = TC_W4_SERVICE_RESULT;
                    gatt_client_discover_primary_services_by_uuid16(handle_gatt_client_event, connection_handle, ORG_BLUETOOTH_SERVICE_ENVIRONMENTAL_SENSING);
                    break;
                default:
                    break;
            }
            break;
        case HCI_EVENT_DISCONNECTION_COMPLETE:
            // unregister listener
            connection_handle = HCI_CON_HANDLE_INVALID;
            if (listener_registered){
                listener_registered = false;
                gatt_client_stop_listening_for_characteristic_value_updates(&notification_listener);
            }
            printf("Disconnected %s\n", bd_addr_to_str(server_addr));
            if (state == TC_OFF) break;
            client_start();
            break;
        default:
            break;
    }
}

static void heartbeat_handler(struct btstack_timer_source *ts) {
    // Invert the led
    static bool quick_flash;
    static bool led_on = true;

    led_on = !led_on;
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led_on);
    if (listener_registered && led_on) {
        quick_flash = !quick_flash;
    } else if (!listener_registered) {
        quick_flash = false;
    }

    // Restart timer
    btstack_run_loop_set_timer(ts, (led_on || quick_flash) ? LED_QUICK_FLASH_DELAY_MS : LED_SLOW_FLASH_DELAY_MS);
    btstack_run_loop_add_timer(ts);
}
