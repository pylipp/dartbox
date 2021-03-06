/*
 * PIN SETUP
 * ARDUINO to LCD
 * - A4 to SDA
 * - A5 to SCL
 * - 5V to VCC
 * - GND to GND
 *
 * ARDUINO to Keypad
 * - see pin assignment, columns from the right keypad pinouts
 */

#include <Wire.h>
#include <Keypad.h>
#include <LiquidCrystal_I2C.h>

#include <ros.h>
#include <std_msgs/String.h>
#include <rosserial_arduino/Test.h>

const uint8_t lcd_cols = 20;
const uint8_t lcd_rows = 4;
LiquidCrystal_I2C lcd(0x27, lcd_cols, lcd_rows);

const uint8_t ROWS = 4;
const uint8_t COLS = 4;
char keys[ROWS][COLS] = {
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'*','0','#','D'}
};
uint8_t colPins[COLS] = {5, 4, 3, 2}; //connect to the row pinouts of the keypad
uint8_t rowPins[ROWS] = {9, 8, 7, 6}; //connect to the column pinouts of the keypad

Keypad keypad = Keypad( makeKeymap(keys), rowPins, colPins, ROWS, COLS );

ros::NodeHandle  nh;
// lock to avoid a callback function being executed while another one is running
bool lock = false;
using rosserial_arduino::Test;

#define MESSAGE_LEN 16
char buffer[MESSAGE_LEN];

#define LCD_ROW_ERROR 0
#define LCD_ROW_INPUT 1
#define LCD_ROW_INFO_VISIT 2
#define LCD_ROW_INFO_FINISHES 3

void clearLcdRow(uint8_t n) {
  lcd.setCursor(0, n);
  // 20 whitespaces...
  lcd.print("                    ");
  lcd.setCursor(0, n);
}

void callback(const Test::Request & req, Test::Response & res){
  if (lock) return;
  lock = true;

  char key;
  int c = 0;

  clearLcdRow(LCD_ROW_INPUT);
  lcd.print(req.input);

  while (true) {
    // in order to avoid losing sync with the serial node
    nh.spinOnce();

    key = keypad.getKey();

    if (key) {
      if (key == '*') {
        // clear any errors on confirmation
        clearLcdRow(LCD_ROW_ERROR);
        clearLcdRow(LCD_ROW_INFO_FINISHES);

        // send response
        buffer[c] = (char)0;
        res.output = buffer;
        break;
      }

      buffer[c] = key;
      lcd.print(key);
      c++;
    }

    delay(10);
  }
  lock = false;
}

void print_info_callback(const std_msgs::String& msg) {
  if (lock) return;

  char indicator = msg.data[0];
  int row = 0;

  if (indicator == '!') {
    row = LCD_ROW_ERROR;
  }
  else if (indicator == '?') {
    row = LCD_ROW_INFO_FINISHES;
  }
  else if (indicator == '%') {
    row = LCD_ROW_INFO_VISIT;
  }
  else 
    return;

  clearLcdRow(row);
  lcd.print(msg.data + 1);
}

ros::ServiceServer<Test::Request, Test::Response> server("get_input", &callback);
ros::Subscriber<std_msgs::String> print_info_subscriber("print_info",
    print_info_callback);

void setup()
{
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, LCD_ROW_ERROR);
  lcd.print("   >==PYDARTS==>   ");
  lcd.setCursor(0, LCD_ROW_INPUT);
  lcd.print("  ... starting ...  ");

  nh.initNode();
  nh.subscribe(print_info_subscriber);
  nh.advertiseService(server);
}

void loop()
{
  nh.spinOnce();
  delay(10);
}
