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

/// @file contains the TFTdisplay class
/// @brief TFTdisplay Class
/// @details This class contains everything needed for the 3.5" TFT display for the R2D2 project.

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

  /// @brief removes one boat from player
  void removeBoats(){
    boats--;
    updateScreen();
  }

  /// @brief adds one shot to player
  void addShot(){
    shots++;
  }

  /// @brief adds one hit to player
  void addHit(){
    hits++;
    addShot();
    updateScreen();
  }

  /// @brief adds one miss to player
  void addMiss(){
    misses++;
    addShot();
    updateScreen();
  }

//  Enemy ships ==================================================================

  /// @brief removes one boat from enemy
  void removeEnemyBoats(){
    enemyboats--;
    updateScreen();
  }

  /// @brief adds one shot to enemy
  void addEnemyShot(){
    enemyshots++;
  }

  /// @brief adds one hit to enemy
  void addEnemyHit(){
    enemyhits++;
    addEnemyShot();
    updateScreen();
  }

  /// @brief adds one miss to enemy
  void addEnemyMiss(){
    enemymisses++;
    addEnemyShot();
    updateScreen();
  }

// Setup and Update ==============================================================

  /// @brief Setup the TFT display needs to run once.
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

  /// @brief Updates the TFT display by calling all draw functions.
  void updateScreen(){
    tft.fillScreen(OCEAN);
    drawBoat();
    drawZeeslag();
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

// Reset function =========================================================

  /// @brief Resets all values to 0, gets called from display.
  void resetTFT(){
    boats = 5;
    shots = 0;
    hits = 0;
    misses = 0;
    enemyboats = 5;
    enemyshots = 0;
    enemyhits = 0;
    enemymisses = 0;
    text = "";
  }

// Position ===============================================================

  /// @brief Sets the position of the joystick in string.
  void setPositionJoystick(int x, int y){
    String pos[2][10] = {{"A", "B", "C", "D", "E", "F", "G", "H", "I", "J"}, {"1", "2", "3", "4", "5", "6", "7", "8", "9", "10"}};
    joystick = pos[0][x] + pos[1][y];
    updateScreen();
  }

  /// @brief Draws the position of the joystick.
  void drawPositionJoystick(){
    tft.setCursor(24,340);
    tft.setTextColor(GREEN);
    tft.setTextSize(4);
    String toPrint = "Location:" + joystick;
    tft.println(toPrint);
  }

// Zeeslag titel ==========================================================

  /// @brief Draws the title (Top bit)
  void drawZeeslag(){
    tft.fillRect(11,11,298,40,RED);
    tft.drawRect(10,10,300,42,YELLOW);
    tft.setCursor(124,25);
    tft.setTextColor(WHITE);
    tft.setTextSize(2);
    tft.println("ZEESLAG");
  }

// Text ===================================================================

  /// @brief Sets the text which is going to get drawn.
  void setText( String textToDraw ){
    text = textToDraw;
    updateScreen();
  }

  /// @brief Draws the text
  void drawText(){
    tft.setCursor(24,400);
    tft.setTextColor(ENEMY);
    tft.setTextSize(3);
    tft.println(text);
  }

// Player draws ============================================================

  /// @brief Draws player boats
  void drawBoats(){
    tft.setCursor(24,80);
    tft.setTextColor(GREEN);
    tft.setTextSize(2);
    tft.println("Player Boats:");
    tft.setCursor(220, 80);
    tft.println(boats);
  }

  /// @brief Draws player shots
  void drawShots(){
    tft.setCursor(24,100);
    tft.setTextColor(GREEN);
    tft.setTextSize(2);
    tft.println("Player shots:");
    tft.setCursor(220, 100);
    tft.println(shots);
  }

  /// @brief Draws player hits
  void drawHits(){
    tft.setCursor(24,120);
    tft.setTextColor(GREEN);
    tft.setTextSize(2);
    tft.println("Player Hits:");
    tft.setCursor(220, 120);
    tft.println(hits);
  }

  /// @brief Draws player misses
  void drawMisses(){
    tft.setCursor(24,140);
    tft.setTextColor(GREEN);
    tft.setTextSize(2);
    tft.println("Player Misses:");
    tft.setCursor(220, 140);
    tft.println(misses);
  }

// Enemy draws ============================================================

  /// @brief Draws enemy boats
  void drawEnemyBoats(){
    tft.setCursor(24,170);
    tft.setTextColor(ENEMY);
    tft.setTextSize(2);
    tft.println("Enemy Boats:");
    tft.setCursor(220, 170);
    tft.println(enemyboats);
  }

  /// @brief Draws enemy shots
  void drawEnemyShots(){
    tft.setCursor(24,190);
    tft.setTextColor(ENEMY);
    tft.setTextSize(2);
    tft.println("Enemy Shots:");
    tft.setCursor(220, 190);
    tft.println(enemyshots);
  }

  /// @brief Draws enemy hits
  void drawEnemyHits(){
    tft.setCursor(24,210);
    tft.setTextColor(ENEMY);
    tft.setTextSize(2);
    tft.println("Enemy Hits:");
    tft.setCursor(220, 210);
    tft.println(enemyhits);
  }

  /// @brief Draws enemy misses
  void drawEnemyMisses(){
    tft.setCursor(24,230);
    tft.setTextColor(ENEMY);
    tft.setTextSize(2);
    tft.println("Enemy Misses:");
    tft.setCursor(220, 230);
    tft.println(enemymisses);
  }


// Background =============================================================================

  /// @brief Draws the background
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
