/*
 * ========================================
 * FLAPPY BIRD - Single Button Game
 * ========================================
 * 
 * Hardware:
 * - Arduino Mega 2560
 * - OLED Display 128x64 I2C (SDA=20, SCL=21)
 * 
 * ========================================
 */

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <EEPROM.h>

// ===== HARDWARE CONFIG =====
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// ===== GAME OBJECTS =====
struct Bird {
  float y;
  float velocity;
  int size;
};

struct Pipe {
  int x;
  int gapY;
  int gapSize;
  bool passed;
  bool active;
};

Bird bird;
Pipe pipes[3];

// ===== GAME STATE =====
int score = 0;
int highScore = 0;
bool gameOver = false;
unsigned long lastPipeSpawn = 0;

// ===== PHYSICS CONSTANTS =====
#define GRAVITY -0.35
#define JUMP_POWER -2.0
#define PIPE_SPEED 2
#define PIPE_GAP 24
#define PIPE_WIDTH 10
int PIPE_INTERVAL = 2500;  // Variabile invece di #define
#define BASE_PIPE_INTERVAL 2500
#define MAX_PIPE 20 //350

// ===== TIME TRACKING =====
unsigned long gameStartTime = 0;
unsigned long completionTime = 0;
unsigned long bestTime = 999999;  // Infinito iniziale (in ms)
int totalPipesPassed = 0;
//bool hasWon = false;

// ===== SPEED BOOST VARIABLES =====
bool speedBoostActive = false;
unsigned long speedBoostStartTime = 0;
#define SPEED_BOOST_DURATION 100  // 500ms = 0.5 secondi
#define SPEED_BOOST_MULTIPLIER 4 // 4x più veloce


//DEFINISCO LA TABELLA Q CHE MI SERVIRÀ PER FAR APPRENDERE L'AI
// ===== Q-LEARNING SETUP =====

// Costanti discretizzazione
#define Y_BUCKETS 8
#define DIST_BUCKETS 8
#define GAP_BUCKETS 8
#define TOTAL_STATES (Y_BUCKETS * DIST_BUCKETS * GAP_BUCKETS)  // 192
#define NUM_ACTIONS 3  // SALTA, NON SALTA, ACCELLERO

// ===== Q-TABLE (INT invece di FLOAT!) =====
int Q[TOTAL_STATES][NUM_ACTIONS];  // 768 bytes invece di 1536!

// ===== HYPERPARAMETERS (adattati per INT) =====
// Learning rate = 15 significa 15/100 = 0.15
int learningRate = 15;         // α × 100 (es: 15 = 0.15)
int discountFactor = 95;       // γ × 100 (es: 95 = 0.95)
int epsilon = 0;              // ε × 100 (es: 40 = 0.40 = 40%)
int epsilonDecay = 996;        // decay × 1000 (es: 996 = 0.996)
int epsilonMin = 1;            // min ε × 100 (es: 1 = 0.01 = 1%)

// ===== TRAINING VARIABLES =====
int episode = 0;
int bestScore = 0;
bool aiMode = false;
bool trainingMode = false;
int count = 1;

// ===== FUNZIONI Q-TABLE =====

void initQTable() {
  Serial.println("Inizializzando Q-Table (INT)...");
  
  for(int i = 0; i < TOTAL_STATES; i++) {
    for(int j = 0; j < NUM_ACTIONS; j++) {
      Q[i][j] = 0;  // Tutti a zero (int)
    }
  }
  
  Serial.print("Q-Table: ");
  Serial.print(TOTAL_STATES);
  Serial.print(" stati × ");
  Serial.print(NUM_ACTIONS);
  Serial.println(" azioni");
  Serial.print("Memoria: ");
  Serial.print(TOTAL_STATES * NUM_ACTIONS * sizeof(int));
  Serial.print(" bytes (era ");
  Serial.print(TOTAL_STATES * NUM_ACTIONS * sizeof(float));
  Serial.println(" con float)");
}

int getStateIndex() {
  int nearestPipeIdx = -1;
  int minDist = 9999;
  
  for(int i = 0; i < 3; i++) {
    if(pipes[i].active && pipes[i].x > 10) {
      int dist = pipes[i].x - 10;
      if(dist >= -5 && dist < minDist) {
        minDist = dist;
        nearestPipeIdx = i;
      }
    }
  }
  
  if(nearestPipeIdx == -1) {
    return 0;
  }
  
  // Discretizza
  int yBucket = constrain((int)(bird.y / (SCREEN_HEIGHT / Y_BUCKETS)), 0, Y_BUCKETS - 1);
  int distBucket = constrain(minDist / 16, 0, DIST_BUCKETS - 1);
  
  int gapBucket;
  if(pipes[nearestPipeIdx].gapY < SCREEN_HEIGHT / 3) {
    gapBucket = 0;
  } else if(pipes[nearestPipeIdx].gapY < 2 * SCREEN_HEIGHT / 3) {
    gapBucket = 1;
  } else {
    gapBucket = 2;
  }
  
  int stateIndex = (yBucket * DIST_BUCKETS * GAP_BUCKETS) + 
                   (distBucket * GAP_BUCKETS) + 
                   gapBucket;
  
  return constrain(stateIndex, 0, TOTAL_STATES - 1);
}

void printStateInfo(int stateIndex) {
  int gap = stateIndex % GAP_BUCKETS;
  int dist = (stateIndex / GAP_BUCKETS) % DIST_BUCKETS;
  int birdY = stateIndex / (DIST_BUCKETS * GAP_BUCKETS);
  
  Serial.print("Stato ");
  Serial.print(stateIndex);
  Serial.print(": Y=");
  Serial.print(birdY);
  Serial.print(", D=");
  Serial.print(dist);
  Serial.print(", G=");
  Serial.print(gap);
  Serial.print(" | Q[SALTA]=");
  Serial.print(Q[stateIndex][0]);
  Serial.print(", Q[NO]=");
  Serial.println(Q[stateIndex][1]);
}

// Sceglie azione basandosi su Q-table (con epsilon-greedy)
int scegliAzione(int stato) {
  
  // EXPLORATION: ogni tanto prova azioni casuali (per scoprire cose nuove)
  if(random(100) < epsilon) {
    // Esplora: azione casuale
    int azioneRandom = random(3);  // 0 o 1 casuale <- la devo modificare se voglio che provi a generare casualmente anche lo scatto
    
    Serial.print("EXPLORE: azione random = ");
    Serial.println(azioneRandom);
    
    return azioneRandom;
  }
  
  // EXPLOITATION: usa quello che ha imparato dalla Q-table
  else {
    // Guarda Q-table e scegli azione migliore
    int valoreSalta = Q[stato][0];      // Valore se salta
    int valoreNonSalta = Q[stato][1];   // Valore se non salta
    int valoreSpeed = Q[stato][2];       // Valore scatto 

    int azioneMigliore = 0;
    
    if(valoreSalta > valoreNonSalta && valoreSalta > valoreSpeed) {
      azioneMigliore = 0;  // SALTA è meglio
    } else if (valoreNonSalta > valoreSalta && valoreNonSalta > valoreSpeed){
      azioneMigliore = 1;  // NON SALTARE è meglio (o uguale)
    } else {
      azioneMigliore = 2;
    }
    
    Serial.print("EXPLOIT: stato=");
    Serial.print(stato);
    Serial.print(" | Q[S]=");
    Serial.print(valoreSalta);
    Serial.print(", Q[N]=");
    Serial.print(valoreNonSalta);
    Serial.print(" → azione=");
    Serial.println(azioneMigliore);
    
    return azioneMigliore;
  }
}

/*
int calcolaReward() {
  
  int halfAltezza = 32;
  int distanzaDalCentro = abs(halfAltezza - bird.y);
  
  // CALCOLO DEL BONUS DI POSIZIONE
  // Invertiamo la logica:
  // Se distanza è 0 (al centro) -> Bonus = 32
  // Se distanza è 32 (ai bordi) -> Bonus = 0
  int positionBonus = (halfAltezza - distanzaDalCentro) *5; 

  // --- MORTE ---
  if(gameOver) {
    // Quando muore, non ci interessa dove si trova, la punizione deve essere secca.
    // Togliere il bonus qui evita calcoli inutili.
    Serial.println("REWARD: -1000 (MORTE!)");
    return REWARD_DEATH; 
  }
  
  // --- PASSATO UNA PIPE ---
  for(int i = 0; i < 3; i++) {
    if(pipes[i].active && pipes[i].passed) {
      int justPassedX = pipes[i].x + PIPE_WIDTH;
      if(justPassedX >= 8 && justPassedX <= 12) {
        Serial.println("REWARD: +100 (PASSATO PIPE!)");
        
        // Qui diamo il premio GRANDE + il piccolo bonus se l'ha passata stando al centro
        return REWARD_PASS_PIPE + positionBonus;
      }
    }
  }
  
  // --- ANCORA VIVO ---
  // Qui la magia: Base (es. 10) + Bonus (0-32).
  // Risultato: tra 10 (ai bordi) e 42 (al centro). Sempre positivo!
  Serial.print("REWARD: VIVO + Pos: ");
  Serial.println(positionBonus);
  
  return REWARD_STAY_ALIVE + positionBonus; 
}*/

// ===== REWARD VALUES (scalati ×10) =====
#define REWARD_PASS_PIPE 1000  // Era +10.0
#define REWARD_STAY_ALIVE -10  // Era +1.0
#define REWARD_DEATH -10000     // Era -100.0

int calcolaReward() {
  if(gameOver) return -100;
  
  for(int i = 0; i < 3; i++) {
    if(pipes[i].active && pipes[i].passed) {
      int justPassedX = pipes[i].x + PIPE_WIDTH;
      if(justPassedX >= 8 && justPassedX <= 12) {
        return 100;
      }
    }
  }
  
  // ===== DENSE REWARD =====
  
  float reward = 1.0;  // Base
  
  int nearestPipeIdx = -1;
  int minDist = 9999;
  for(int i = 0; i < 3; i++) {
    if(pipes[i].active && pipes[i].x > 10) {
      int dist = pipes[i].x - 10;
      if(dist < minDist) {
        minDist = dist;
        nearestPipeIdx = i;
      }
    }
  }
  
  if(nearestPipeIdx != -1) {
    int gapCenter = pipes[nearestPipeIdx].gapY;
    int birdCenter = bird.y + (bird.size / 2);
    int distFromGap = abs(gapCenter - birdCenter);
    
    // 1. ALIGNMENT (peso alto)
    float alignmentFactor = 1.0 - (distFromGap / 32.0);  // 0-1
    reward += alignmentFactor * 4;  // Max +4
    
    // 2. PROXIMITY (solo se allineato)
    if(distFromGap < 15) {
      float proximityFactor = 1.0 - (minDist / 128.0);  // 0-1
      reward += proximityFactor * 3;  // Max +3
    }
    
    // 3. VERTICAL CENTERING (stare a metà schermo è safer)
    int screenCenter = SCREEN_HEIGHT / 2;
    int distFromScreenCenter = abs(birdCenter - screenCenter);
    float centeringFactor = 1.0 - (distFromScreenCenter / 32.0);
    reward += centeringFactor * 1;  // Max +1
    
    // 4. EDGE PENALTY
    int distFromTop = bird.y;
    int distFromBottom = SCREEN_HEIGHT - (bird.y + bird.size);
    int minEdgeDist = min(distFromTop, distFromBottom);
    if(minEdgeDist < 10) {
      reward -= (10 - minEdgeDist) * 0.5;  // Max -5
    }
    
    // 5. VELOCITY PENALTY (di caduta)
    float velocityPenalty = abs(bird.velocity) / 10.0;
    reward -= velocityPenalty;  // Max -1 circa

    // ===== 6. BOOST REWARD/PENALTY =====
    if(speedBoostActive) {
      // PENALITÀ se boost attivo VICINO a pipe
      if(minDist < 50) {
        // Molto vicino = molto pericoloso
        float dangerPenalty = (50 - minDist) / 10.0;  // Max -5
        reward -= dangerPenalty;
        
        // Penalità extra se anche disallineato
        if(distFromGap > 10) {
          reward -= 2;  // Boost vicino E disallineato = molto male!
        }
      } 
      // BONUS se boost attivo LONTANO da pipe
      else if(minDist > 80) {
        reward += 2;  // Bravo! Usi boost quando è sicuro
      }
      // Neutro se a distanza media (50-80)
    }
    // Piccola penalità se NON usa boost quando è lontano
    else {
      if(minDist > 90) {
        reward -= 0.5;  // Potresti andare più veloce qui!
      }
    }
  }
  
  return (int)reward;  // Converti a int
}

// ===== SALVATAGGIO EEPROM =====

// ===== EEPROM ADDRESSES =====
#define EEPROM_MAGIC_ADDR 0
#define EEPROM_EPISODE_ADDR 4
#define EEPROM_BEST_ADDR 8
#define EEPROM_EPSILON_ADDR 12
#define EEPROM_Q_START 16
#define MAGIC_NUMBER 0xABCD

void saveQTable() {
  Serial.println("Saving Q-table...");
  
  // Magic number
  EEPROM.put(EEPROM_MAGIC_ADDR, MAGIC_NUMBER);
  
  // Stats
  EEPROM.put(EEPROM_EPISODE_ADDR, episode);
  EEPROM.put(EEPROM_BEST_ADDR, bestScore);
  EEPROM.put(EEPROM_EPSILON_ADDR, epsilon);
  
  // Q-table
  int addr = EEPROM_Q_START;
  for(int i = 0; i < TOTAL_STATES; i++) {
    for(int j = 0; j < NUM_ACTIONS; j++) {
      EEPROM.put(addr, Q[i][j]);
      addr += sizeof(int);
    }
  }
  
  Serial.println("Saved!");
}

void loadQTable() {
  uint16_t magic;
  EEPROM.get(EEPROM_MAGIC_ADDR, magic);
  
  if(magic == MAGIC_NUMBER) {
    Serial.println("Loading Q-table...");
    
    // Stats
    EEPROM.get(EEPROM_EPISODE_ADDR, episode);
    EEPROM.get(EEPROM_BEST_ADDR, bestScore);

    //EEPROM.get(EEPROM_EPSILON_ADDR, epsilon); <- voglio poter modificare io epsilon non che mi viene usato sempre lo stesso
    
    // Q-table
    int addr = EEPROM_Q_START;
    for(int i = 0; i < TOTAL_STATES; i++) {
      for(int j = 0; j < NUM_ACTIONS; j++) {
        EEPROM.get(addr, Q[i][j]);
        addr += sizeof(int);
      }
    }
    
    Serial.print("Loaded! Ep:");
    Serial.print(episode);
    Serial.print(" Best:");
    Serial.print(bestScore);
    Serial.print(" ε:");
    Serial.println(epsilon);
  } else {
    Serial.println("No saved data, initializing fresh...");
    initQTable();
    episode = 0;
    bestScore = 0;
    epsilon = 40;
  }
}


// Stampa primi N valori della Q-table
void printQTable(int numStates) {
  Serial.println("========================================");
  Serial.println("Q-TABLE VALUES");
  Serial.println("========================================");
  Serial.println("State | JUMP  | NO_JUMP | SCATTO");
  Serial.println("------|-------|---------|--------");
  
  for(int i = 0; i < numStates; i++) {
    // Formatta output
    if(i < 10) Serial.print("  ");
    else if(i < 100) Serial.print(" ");
    
    Serial.print(i);
    Serial.print("   | ");
    
    // Valore JUMP
    if(Q[i][0] >= 0) Serial.print(" ");
    if(abs(Q[i][0]) < 10) Serial.print("  ");
    else if(abs(Q[i][0]) < 100) Serial.print(" ");
    Serial.print(Q[i][0]);
    
    Serial.print("  | ");
    
    // Valore NO_JUMP
    if(Q[i][1] >= 0) Serial.print(" ");
    if(abs(Q[i][1]) < 10) Serial.print("  ");
    else if(abs(Q[i][1]) < 100) Serial.print(" ");
    Serial.print(Q[i][1]);
    
    Serial.println();
  }
  
  Serial.println("========================================");
}

void resetQTable() {
  Serial.println("RESET Q-Table!");
  initQTable();
  episode = 0;
  bestScore = 0;
  epsilon = 40;
}


// ===== SETUP =====
void setup() {
  Serial.begin(9600);

  /*
  //reset eemprom
  for (int i = 0 ; i < EEPROM.length() ; i++) {
    // Scrive 0 in ogni singola cella
    EEPROM.write(i, 0);
  }
  */

  // Initialize display
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("Display FAILED!"));
    while(1);
  }
  
  // CARICA Q-TABLE DA EEPROM
  loadQTable();
  //Stampa
  printQTable(100);
  delay(500);
  // Show splash screen
  showSplash();
  
  // Start game
  initGame();
  
  Serial.println("=== FLAPPY BIRD ===");
  Serial.println("Press button to jump!");
}

// ===== SPLASH SCREEN =====
void showSplash() {
  display.clearDisplay();
  display.setTextSize(3);
  display.setTextColor(WHITE);
  display.setCursor(10, 15);
  display.println("FLAPPY");
  display.setTextSize(2);
  display.setCursor(30, 40);
  display.println("BIRD");
  display.display();
  
  delay(2000);
}



// ===== INIT GAME =====
void initGame() {
  // Reset bird
  bird.y = SCREEN_HEIGHT / 2;
  bird.velocity = 0;
  bird.size = 4;
  
  // Reset pipes
  for(int i = 0; i < 3; i++) {
    pipes[i].active = false;
    pipes[i].passed = false;
  }
  
  // Reset game state
  score = 0;
  gameOver = false;
  //lastPipeSpawn = millis();
  lastPipeSpawn = millis() - PIPE_INTERVAL; //per far spawnare subito la pipe

  saveQTable();
  
  Serial.println("Game started!");
}

// ===== MAIN LOOP =====
void loop() {
  // ho tolto l'if per vedere se era game over perché anche se è gameover voglio entrare qui

  //in questo caso è l'AI che gioca, io non ho potere
  int oldState = getStateIndex(); //prendo lo stato attuale nel quale si trova il BIRD
  int action = scegliAzione(oldState); //uso lo stato attuale come indice per andare nella tebella a un determinato indice e scelgo l'azione che ha lo score maggiore
  
  if (action == 0) {bird.velocity = JUMP_POWER;} //faccio fare il salto al BIRD
  if(!speedBoostActive) 
  {  // ← Solo se non era già attivo!
    speedBoostActive = true;
    speedBoostStartTime = millis();
    
    // Calcola quanto tempo è passato dall'ultimo spawn
    unsigned long timeSinceLastSpawn = millis() - lastPipeSpawn;
    
    // Calcola quanto dovrebbe essere passato con il nuovo intervallo
    // Proporzione: timeSinceLastSpawn / BASE_PIPE_INTERVAL = x / NEW_PIPE_INTERVAL
    unsigned long newInterval = BASE_PIPE_INTERVAL / SPEED_BOOST_MULTIPLIER;
    unsigned long adjustedTime = (timeSinceLastSpawn * newInterval) / BASE_PIPE_INTERVAL;
    
    // Aggiusta lastPipeSpawn
    lastPipeSpawn = millis() - adjustedTime;
    
    // Cambia intervallo
    PIPE_INTERVAL = newInterval;
  }

  // Controlla se il boost è scaduto
  if(speedBoostActive && millis() - speedBoostStartTime > SPEED_BOOST_DURATION) {
    speedBoostActive = false;
    
    // ===== DISATTIVA BOOST =====
    // Calcola quanto tempo è passato dall'ultimo spawn
    unsigned long timeSinceLastSpawn = millis() - lastPipeSpawn;
    
    // Aggiusta proporzione inversa
    unsigned long adjustedTime = (timeSinceLastSpawn * BASE_PIPE_INTERVAL) / PIPE_INTERVAL;
    
    // Aggiusta lastPipeSpawn
    lastPipeSpawn = millis() - adjustedTime;
    
    // Ripristina intervallo
    PIPE_INTERVAL = BASE_PIPE_INTERVAL;
  }
  //sennò non gli faccio fare nulla

  updatePhysics(); //aggiorno la fisica perché tanto funziona sia se AI decide di saltare sia se decide di stare ferma
  updatePipes();
  checkCollisions();
  //sono tutti e tre utili per far continuare il gioco

  //adesso devo capire se lo stato in cui mi trovo è migliore o peggiore rispetto al precedente
  int newState = getStateIndex();
  //su questo nuovo state devo calcolare uno score per capire se sta andando meglio o peggio

  int reward = calcolaReward();

  //allo stato attuale nel quale mi sono mosso è affidato un punteggio a premere o no il pulsante, prendo il massimo tra i due ed è quello che ricevo se vado in questa situazione
  //se una azione e molto buona da eseguire vuol dire che tutte le volte che l'ho eseguita ho guadagnato punti -> se uno stato a una delle due azioni con valori molto alti allora vuol dire che è sicuro che andrò a prendere punti in questa situazione 
  int maxNextQ = max(max(Q[newState][0], Q[newState][1]), Q[newState][2]); //<- devo modificare se aggiungo azioni così capisco quale è l'azinoe migliore da compiere

  //ora che mi sono calcolato la reward per passare dallo stato old a quello new e quanto è buono lo stato new tramite "maxNextQ" è il momento di andare ad aggiornare il valore della azione che mi ha portato dallo stato old a questo attuale
  Q[oldState][action] += (learningRate * (reward + (discountFactor * maxNextQ / 100) - Q[oldState][action])) / 100;  

  if (gameOver) handleGameOver();  
  // Draw everything
  drawGame();
}

/*
void handleInputAI()  //da qui ricavo l'indice che devo andare a vedere all'interno della tabella Q dalla quale poi prendo l'azione da compiere
{
  //indice della tabella dipende da 3 fattori diversi: 1. altezza del bird 2. distanza dalla pipe lungo x 3. distanza dalla pipe lungo y
  int Zona_Bird_Y = (int)(bird.y / 8); //lungo y divido le posizioni che possono essere ricoperte dal BIRD in 8 sezioni

  int distanza = pipes[0].x - 10;
  int Zona_Distanza = (int)(distanza / 8);  //per la distanza del BIRD dalla pipe divido lo spazio in 8 sezioni

  //per quanto riguarda il GAP verticale la situazione si fa un po più complicata.
  //GAP verticale può essere ovunque -> per semplicità iniziale suppongo si possa trovare solamente in 3 zone
  int Zona_Gap = 0;
  if (pipes[0].gapY < 21) Zona_Gap = 0;
  else if (pipes[0].gapY < 43) Zona_Gap = 1;
  else Zona_Gap = 2;

  //tramite queste tre variabili riesco a ricavare le informazioni principali che servono all'AI per decidere se saltare o meno
  //a questo punto queste informazioni mi sono necessarie per identificare la situazione in cui mi trovo e andare a vedere all'interno della tabella Q quale è l'azione migliore che posso compiere in questa situazione
  int STATO = (Zona_Bird_Y * 8 * 3) + (Zona_Distanza * 3) + Zona_Gap;
  //ora andrò a cercare all'interno della tavella Q l'elemento all'indice STATO e vedo quale è l'azione migliore che posso compiere
}*/

// ===== PHYSICS UPDATE =====
void updatePhysics() {
  // Apply gravity
  bird.velocity -= GRAVITY;
  bird.y += bird.velocity;
  
  // Check top boundary
  if(bird.y < 0) {
    bird.y = 0;
    bird.velocity = 0;
    gameOver = true;
  }
  
  // Check bottom boundary
  if(bird.y + bird.size > SCREEN_HEIGHT) {
    bird.y = SCREEN_HEIGHT - bird.size;
    bird.velocity = 0;
    gameOver = true;
  }
}

// ===== PIPES UPDATE =====
void updatePipes() {
  int currentSpeed = speedBoostActive ? (PIPE_SPEED * SPEED_BOOST_MULTIPLIER) : PIPE_SPEED;
  // Move existing pipes
  for(int i = 0; i < 3; i++) {
    if(pipes[i].active) {
      pipes[i].x -= currentSpeed;
      
      // Remove pipe when off screen
      if(pipes[i].x + PIPE_WIDTH < 0) {
        pipes[i].active = false;
      }
      
      // Score point when bird passes pipe
      if(!pipes[i].passed && pipes[i].x + PIPE_WIDTH < 10) {
        pipes[i].passed = true;
        score++;
        
        // Update high score
        if(score > highScore) {
          highScore = score;
        }
        
        Serial.print("Score: ");
        Serial.println(score);
      }
    }
  }
  
  // Spawn new pipe
  if(millis() - lastPipeSpawn > PIPE_INTERVAL) {
    for(int i = 0; i < 3; i++) {
      if(!pipes[i].active) {
        pipes[i].active = true;
        pipes[i].x = SCREEN_WIDTH;
        pipes[i].gapY = random(PIPE_GAP / 2 + 8, SCREEN_HEIGHT - PIPE_GAP / 2 - 8);
        pipes[i].gapSize = PIPE_GAP;
        pipes[i].passed = false;
        lastPipeSpawn = millis();
        break;
      }
    }
  }
}

// ===== COLLISION CHECK =====
void checkCollisions() {
  int birdLeft = 10;
  int birdRight = 10 + bird.size;
  int birdTop = (int)bird.y;
  int birdBottom = (int)bird.y + bird.size;
  
  for(int i = 0; i < 3; i++) {
    if(pipes[i].active) {
      // Check if bird overlaps with pipe horizontally
      if(birdRight > pipes[i].x && birdLeft < pipes[i].x + PIPE_WIDTH) {
        
        int gapTop = pipes[i].gapY - pipes[i].gapSize / 2;
        int gapBottom = pipes[i].gapY + pipes[i].gapSize / 2;
        
        // Check if bird hits top or bottom pipe
        if(birdTop < gapTop || birdBottom > gapBottom) {
          gameOver = true;
          
          Serial.print("Game Over! Final score: ");
          Serial.println(score);
        }
      }
    }
  }
}

// ===== GAME OVER =====
void handleGameOver() {
  // Wait a moment
  count++;
  initGame();
}

// ===== DRAW EVERYTHING =====
void drawGame() {
  display.clearDisplay();
  
  // Draw bird
  int birdX = 10;
  
  // Bird body (circle)
  display.fillCircle(birdX + 2, (int)bird.y + 2, 3, WHITE);
  
  // Bird eye
  display.drawPixel(birdX + 3, (int)bird.y + 1, BLACK);
  
  // Bird beak (changes direction based on velocity)
  if(bird.velocity < -1) {
    // Flying up
    display.drawLine(birdX + 4, (int)bird.y, birdX + 6, (int)bird.y - 1, WHITE);
  } else if(bird.velocity > 1) {
    // Falling down
    display.drawLine(birdX + 4, (int)bird.y + 4, birdX + 6, (int)bird.y + 5, WHITE);
  } else {
    // Neutral
    display.drawLine(birdX + 4, (int)bird.y + 2, birdX + 6, (int)bird.y + 2, WHITE);
  }
  
  // Draw pipes
  for(int i = 0; i < 3; i++) {
    if(pipes[i].active) {
      int gapTop = pipes[i].gapY - pipes[i].gapSize / 2;
      int gapBottom = pipes[i].gapY + pipes[i].gapSize / 2;
      
      // Top pipe
      display.fillRect(pipes[i].x, 0, PIPE_WIDTH, gapTop, WHITE);
      
      // Bottom pipe
      display.fillRect(pipes[i].x, gapBottom, PIPE_WIDTH, 
                      SCREEN_HEIGHT - gapBottom, WHITE);
      
      // Pipe caps (decorative)
      display.fillRect(pipes[i].x - 1, gapTop - 3, PIPE_WIDTH + 2, 3, WHITE);
      display.fillRect(pipes[i].x - 1, gapBottom, PIPE_WIDTH + 2, 3, WHITE);
    }
  }
  
  // Draw score (big, centered at top)
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(SCREEN_WIDTH / 2 - 8, 3);
  display.print(score);
  
  // Draw count (small, top right)
  display.setTextSize(1);
  display.setCursor(10, 2);  // In alto a destra
  display.print(count);

  // ===== HIGH SCORE (piccolo, destra) =====
  display.setTextSize(1);
  display.setCursor(96, 2);  // ← In alto a destra
  display.print(highScore);
  
  // Game Over overlay
  if(gameOver) {
    // Semi-transparent background box
    display.fillRect(15, 22, 98, 20, BLACK);
    display.drawRect(15, 22, 98, 20, WHITE);
    display.drawRect(16, 23, 96, 18, WHITE);
    
    // Text
    display.setTextSize(2);
    display.setCursor(20, 26);
    display.println("GAME OVER");
    
    // Instructions
    display.setTextSize(1);
    display.setCursor(25, 48);
    display.println("Press to retry");
  }
  
  display.display();
}