#ifndef DISPLAY_HPP
#define DISPLAY_HPP
#include <Adafruit_GFX.h>
#include <MCUFRIEND_kbv.h>
MCUFRIEND_kbv tft;

#define BLACK   0x0000
#define BLUE    0x0010
#define RED     0xF800
#define ENEMY   0xFFE0
#define GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF
#define OCEAN   0x60FF

class TFTdisplay {
private:
  int boats = 5;
  int shots = 0;
  int hits = 0;
  int misses = 0;

  int enemyboats = 5;
  int enemyshots = 0;
  int enemyhits = 0;
  int enemymisses = 0;

  String text = "";
  String joystick = "A1";

public:
// Player ships =================================================================

  void removeBoats(){
    boats--;
    updateScreen();
  }

  void addShot(){
    shots++;
  }

  void addHit(){
    hits++;
    addShot();
    updateScreen();
  }

  void addMiss(){
    misses++;
    addShot();
    updateScreen();
  }

//  Enemy ships ==================================================================

  void removeEnemyBoats(){
    enemyboats--;
    updateScreen();
  }

  void addEnemyShot(){
    enemyshots++;
  }

  void addEnemyHit(){
    enemyhits++;
    addEnemyShot();
    updateScreen();
  }

  void addEnemyMiss(){
    enemymisses++;
    addEnemyShot();
    updateScreen();
  }

// Setup and Update ==============================================================

  void setupScreen() {
    // Reading TFT ID:
    uint16_t ID=tft.readID();
    Serial.begin(9600);
    Serial.println(ID);
    //Initializing TFT display:
    tft.begin(ID);
    
    tft.fillScreen(OCEAN);
    tft.fillRect(11,11,298,40,RED);
    tft.drawRect(10,10,300,42,YELLOW);
    tft.setCursor(124,25);
    tft.setTextColor(WHITE);
    tft.setTextSize(2);
    tft.println("ZEESLAG");
    updateScreen();
  }

  void updateScreen(){
    tft.fillScreen(OCEAN);
    drawBoat();
    tft.fillRect(11,11,298,40,RED);
    tft.drawRect(10,10,300,42,YELLOW);
    tft.setCursor(124,25);
    tft.setTextColor(WHITE);
    tft.setTextSize(2);
    tft.println("ZEESLAG");
    drawBoats();
    drawShots();
    drawHits();
    drawMisses();
    drawEnemyBoats();
    drawEnemyShots();
    drawEnemyHits();
    drawEnemyMisses();
    drawText();
    drawPositionJoystick();
  }

// Position ===============================================================

  void setPositionJoystick(int x, int y){
    String pos[2][10] = {{"A", "B", "C", "D", "E", "F", "G", "H", "I", "J"}, {"1", "2", "3", "4", "5", "6", "7", "8", "9", "10"}};
    joystick = pos[0][x] + pos[1][y];
    updateScreen();
  }

  String getPositionJoystick(){
    return joystick;
  }

  void drawPositionJoystick(){
    tft.setCursor(24,320);
    tft.setTextColor(GREEN);
    tft.setTextSize(3);
    String toPrint = "Locatie: " + joystick;
    tft.println(toPrint);
  }

// Text ===================================================================

  void setText( String textToDraw ){
    text = textToDraw;
    updateScreen();
  }

  void drawText(){
    tft.setCursor(48,360);
    tft.setTextColor(BLACK);
    tft.setTextSize(4);
    tft.println(text);
  }

// Player draws ============================================================

  void drawBoats(){
    tft.setCursor(24,80);
    tft.setTextColor(GREEN);
    tft.setTextSize(2);
    tft.println("Player Boats:");
    tft.setCursor(220, 80);
    tft.println(boats);
  }

  void drawShots(){
    tft.setCursor(24,100);
    tft.setTextColor(GREEN);
    tft.setTextSize(2);
    tft.println("Player shots:");
    tft.setCursor(220, 100);
    tft.println(shots);
  }

  void drawHits(){
    tft.setCursor(24,120);
    tft.setTextColor(GREEN);
    tft.setTextSize(2);
    tft.println("Player Hits:");
    tft.setCursor(220, 120);
    tft.println(hits);
  }

  void drawMisses(){
    tft.setCursor(24,140);
    tft.setTextColor(GREEN);
    tft.setTextSize(2);
    tft.println("Player Misses:");
    tft.setCursor(220, 140);
    tft.println(misses);
  }

// Enemy draws ============================================================

  void drawEnemyBoats(){
    tft.setCursor(24,170);
    tft.setTextColor(ENEMY);
    tft.setTextSize(2);
    tft.println("Enemy Boats:");
    tft.setCursor(220, 170);
    tft.println(enemyboats);
  }

  void drawEnemyShots(){
    tft.setCursor(24,190);
    tft.setTextColor(ENEMY);
    tft.setTextSize(2);
    tft.println("Enemy Shots:");
    tft.setCursor(220, 190);
    tft.println(enemyshots);
  }

  void drawEnemyHits(){
    tft.setCursor(24,210);
    tft.setTextColor(ENEMY);
    tft.setTextSize(2);
    tft.println("Enemy Hits:");
    tft.setCursor(220, 210);
    tft.println(enemyhits);
  }

  void drawEnemyMisses(){
    tft.setCursor(24,230);
    tft.setTextColor(ENEMY);
    tft.setTextSize(2);
    tft.println("Enemy Misses:");
    tft.setCursor(220, 230);
    tft.println(enemymisses);
  }


// Background =============================================================================

  void drawBoat(){
    tft.setTextSize(1);
    tft.setTextColor(WHITE);
    tft.setCursor(0, 140);
    tft.println("                                 ____");
    tft.println("                              __(     )____");
    tft.println("                             (_          __)");
    tft.println("                               (_     ___)");
    tft.println("           ____                  (____) ");
    tft.println("        __(     )___");
    tft.println("       (_         __)");
    tft.println("         (_    ___)");
    tft.println("           (___)");
    tft.println("                                    _____");
    tft.println("__                                _(     )_");
    tft.println("   )__                           (__   ____)");
    tft.println("    _ )_                            (____)");
    tft.println("  __    )__                 |>>>");
    tft.println("     ______)               /|                ___");
    tft.println("  _____)                  / |\\           ___(   )__");
    tft.println("                         /  | \\         (_       __)");
    tft.println("                        /   |  \\          (_  ___)");
    tft.println("                       /    |   \\          (___)");
    tft.println("                      /     |    \\");
    tft.println("                     /______|_____\\");
    tft.println("                         ___|__");
    tft.println("                 _______/ooo__\\______/~");
    tft.println("                 \\                   |]");
    tft.println("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~");
    tft.println("~~~~~~~      ~~~~~~~~~~       ~~~~~~ ~~~   ~~  ~~~~~");
    tft.println("~~~ ~~~~~  ~~~   ~~~   ~~~~~~  ~~~~~~~ ~~~~~~~~~   ~~");
    tft.println("~~~  ~~~     ~~~~~~~~~   ~~~~~~~   ~~~~~~~~   ~~~~~~~");
    tft.println("~~~    ~~~       ~~~~~~~~~       ~~~~~       ~~~~");
    tft.println("   ~~~~   ~~         ~~~~~~~     ~~~~~~~~~   ~~~~~~~ ");
    tft.println("   ~~~~~~      ~~~~~~~~~~      ~~~~      ~~~~~   ~~~~");
    tft.println("   ~~   ~~     ~~~~~~~~     ~~                  ~~~~~");
    tft.println("       ~~~~~~~~     ~~~~      ~~~~~~~~~ ~~~~~~ V2B~~~");
    tft.println("~~~~~~~   ~~~~~   ~~~~~~   ~~~~~   ~~~~~  ~~~~ ~~~~~~");
    tft.println("   ~~~~      ~~~~~~~~~~       ~~~~~~ ~~~   ~~  ~~~~~");
    tft.println("  ~~~~~    ~~~   ~~~   ~~~~~~  ~~~~~~~ ~~~~~~~~~   ~~");
    tft.println("     ~~~     ~~~~~~~~~   ~~~~~~~   ~~~~~~~~   ~~~~~~~");
    tft.println("       ~~~       ~~~~~~~~~       ~~~~~       ~~~~");
    tft.println(" ~~~~~   ~~~         ~~~~~~~     ~~~~~~~~~   ~~~~~~~ ");
    tft.println("   ~~~~~~      ~~~~~~~~~~      ~~~~      ~~~~~   ~~~~");
    tft.println(" ~~~~   ~~     ~~~~~~~~     ~~                  ~~~~~");

  }
};

#endif //DISPLAY_HPP
