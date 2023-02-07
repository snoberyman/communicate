/*
 * @file       communicate.pde
 * @author     Yaman
 * @version    V1.0.0
 * @date       7-Feb-2023
 * @brief      Communicate feelings through haply's 2diy force feedback
 */

  /* library imports *****************************************************************************************************/ 
import processing.serial.*;
import static java.util.concurrent.TimeUnit.*;
import java.util.concurrent.*;
/* end library imports *************************************************************************************************/  

/* scheduler definition ************************************************************************************************/ 
private final ScheduledExecutorService scheduler      = Executors.newScheduledThreadPool(1);
/* end scheduler definition ********************************************************************************************/ 

/* device block definitions ********************************************************************************************/
Board             haplyBoard;
Device            widgetOne;
Mechanisms        pantograph;

byte              widgetOneID                         = 5;
int               CW                                  = 0;
int               CCW                                 = 1;
boolean           renderingForce                     = false;
/* end device block definition *****************************************************************************************/



/* framerate definition ************************************************************************************************/
long              baseFrameRate                       = 120;
/* end framerate definition ********************************************************************************************/ 



/* elements definition *************************************************************************************************/

/* Screen and world setup parameters */
float             pixelsPerMeter                      = 4000.0;
float             radsPerDegree                       = 0.01745;
float             pixelsPerCentimeter                 = 40.0;


/* end effector radius in meters */
float             rEE                                 = 0.002;
float             rEEContact                          = 0.006;


/* virtual wall parameters */
PVector           fWall                               = new PVector(0, 0);
float             kWall                               = 800; // N/m

PVector           endSwingWall                       = new PVector(0, 0);
PVector           penWallv1                          = new PVector(0, 0);
PVector           penWallv2                         = new PVector(0, 0);


/* generic data for a 2DOF device */
/* joint space */
PVector           angles                              = new PVector(0, 0);
PVector           torques                             = new PVector(0, 0);

/* task space */
PVector           posEE                               = new PVector(0, 0);
PVector           fEE                                 = new PVector(0, 0); 

/* device graphical position */
PVector           deviceOrigin                        = new PVector(0, 0);

/* World boundaries reference */
final int         worldPixelWidth                     = 1000;
final int         worldPixelHeight                    = 650;

FWorld            world;
float             worldWidth                          = 25.0;  
float             worldHeight                         = 16.0; 

float             edgeTopLeftX                        = 0.0; 
float             edgeTopLeftY                        = 0.0; 
float             edgeBottomRightX                    = worldWidth; 
float             edgeBottomRightY                    = worldHeight;

float             gravityAcceleration                 = 980; //cm/s2

HVirtualCoupling  s;

/* World objects */
FBox              b1;
FBox              g1;


/* graphical elements */
PShape endEffector;
PShape vertical1;
PImage eeImage, targetImage;
/* end elements definition *********************************************************************************************/ 


/* setup section *******************************************************************************************************/
void setup(){
  /* screen size definition */
  size(1000, 650);
  
  /* device setup */
  haplyBoard          = new Board(this, Serial.list()[0], 0);
  widgetOne           = new Device(widgetOneID, haplyBoard);
  pantograph          = new Pantograph();
  
  widgetOne.set_mechanism(pantograph);
  
  widgetOne.add_actuator(1, CCW, 2);
  widgetOne.add_actuator(2, CW, 1);
 
  widgetOne.add_encoder(1, CCW, 241, 10752, 2);
  widgetOne.add_encoder(2, CW, -61, 10752, 1);
  
  widgetOne.device_set_parameters();
    
  hAPI_Fisica.init(this); 
  hAPI_Fisica.setScale(pixelsPerCentimeter); 
  world               = new FWorld();
  /* visual elements setup */
 
  background(255);

  deviceOrigin.add(worldPixelWidth/2, 0);
  
  /* create end effector graphics */
  create_endEffector();
 
  /* create platform */
  b1                  = new FBox(10.0, 0.4);
  b1.setPosition(edgeTopLeftX+worldWidth/1.0-7, edgeTopLeftY+worldHeight/2.0+2.5); 
  b1.setFill(0);
  b1.setNoStroke();
  b1.setStaticBody(true);
  world.add(b1);
  
  /* create bowling pin */
  g1                  = new FBox(1, 2.3);
  g1.setPosition(17,9.1);
  g1.setDensity(10000);
  targetImage = loadImage("target.png");
  targetImage.resize(30,100);
  g1.attachImage(targetImage);
  world.add(g1);

  /* create vertical wall */
  vertical1 = create_wall(-0.08-rEE, 0.05, -0.08-rEE, 0.08);
  
  /* Setup the Virtual Coupling Contact Rendering Technique */
  s                   = new HVirtualCoupling((0.75)); 
  eeImage = loadImage("ball.png"); 
  eeImage.resize(40, 40); 
  s.h_avatar.attachImage(eeImage); 
  s.h_avatar.setSensor(true);

  s.init(world, edgeTopLeftX+worldWidth/2, edgeTopLeftY+2); 
  
  /* World conditions setup */
  world.setGravity((0.0), gravityAcceleration); //1000 cm/(s^2) 
  world.setEdgesRestitution(.1);
  world.setEdgesFriction(0.1);
 
  /* setup framerate speed */
  frameRate(baseFrameRate);
  
  /* setup simulation thread to run at 1kHz */ 
  SimulationThread st = new SimulationThread();
  scheduler.scheduleAtFixedRate(st, 1, 1, MILLISECONDS);
}
/* end setup section ***************************************************************************************************/


/* draw section ********************************************************************************************************/
void draw(){
  /* put graphical code here, runs repeatedly at defined framerate in setup, else default at 60fps: */
  if(renderingForce == false){
    update_animation(angles.x*radsPerDegree, angles.y*radsPerDegree, posEE.x, posEE.y);
  }
}
/* end draw section ****************************************************************************************************/


/* simulation section **************************************************************************************************/
class SimulationThread implements Runnable{
  
  public void run(){
    /* put haptic simulation code here, runs repeatedly at 1kHz as defined in setup */
    
    renderingForce = true;
    
    if(haplyBoard.data_available()){
      /* GET END-EFFECTOR STATE (TASK SPACE) */
      widgetOne.device_read_data();
    
      angles.set(widgetOne.get_device_angles()); 
      posEE.set(widgetOne.get_device_position(angles.array()));
      
      /* check if EE is on the right side of the screen to not use "graphics_to_device" function for sitting the EE position */
      if (posEE.x < -0.002){
          posEE.set(posEE.copy().mult(200)); 
      } 
    }
    
    /* start force feedback calculation */
    if (posEE.x < -0.002){
        s.setToolPosition(edgeTopLeftX+worldWidth/2-(posEE).x, edgeTopLeftY+(posEE).y-7); 
        s.updateCouplingForce();
     
        fEE.set(-s.getVirtualCouplingForceX(), s.getVirtualCouplingForceY());
        fEE.div(100000); //dynes to newtons
        
        world.step(1.0f/1000.0f);
        s.h_avatar.setSensor(false);
    } else {
        posEE.set(device_to_graphics(posEE));
        fWall.set(0, 0);
        
        /*create shaky wall*/
        penWallv1.set((-0.087 - (posEE.x+rEE)), 0);
      
        if((posEE.y > 0.05 && posEE.y < 0.08) && (penWallv1.x < 0 && penWallv1.x > -0.01) ){
           fWall = fWall.add(penWallv1.mult(1000));
         }

       //swing
       if(((posEE.x > -0.08)  && posEE.y > 0.08)){
         endSwingWall.set((0.05 - (posEE.x+rEE)),(0.08 - (posEE.y + rEE)));
         fWall = fWall.add(endSwingWall.mult(-300));  
       }

        fEE = (fWall.copy()).mult(-1);
        fEE.set(graphics_to_device(fEE));
       }

   torques.set(widgetOne.set_device_torques(fEE.array()));
   widgetOne.device_write_torques(); 
   /* end force feedback calculation */

    renderingForce = false;
  }
}
/* end simulation section **********************************************************************************************/


/* helper functions section, place helper functions here ***************************************************************/
void create_endEffector(){
  float rEEAni = pixelsPerMeter * rEE;

  endEffector = createShape(ELLIPSE, deviceOrigin.x, deviceOrigin.y, 2*rEEAni, 2*rEEAni);
  endEffector.setStroke(color(0));
  strokeWeight(5);
  
}


PShape create_wall(float x1, float y1, float x2, float y2){
  x1 = pixelsPerMeter * x1;
  y1 = pixelsPerMeter * y1;
  x2 = pixelsPerMeter * x2;
  y2 = pixelsPerMeter * y2;
  
  return createShape(LINE, deviceOrigin.x + x1, deviceOrigin.y + y1, deviceOrigin.x + x2, deviceOrigin.y+y2);
}

void update_animation(float th1, float th2, float xE, float yE){
  background(255);
 
  xE = pixelsPerMeter * xE;
  yE = pixelsPerMeter * yE;
  
  th1 = 3.14 - th1;
  th2 = 3.14 - th2;
  
  shape(vertical1);
  stroke(0);
  world.draw();

  translate(xE, yE);
  shape(endEffector);
}

PVector device_to_graphics(PVector deviceFrame){
  return deviceFrame.set(-deviceFrame.x, deviceFrame.y);
}


PVector graphics_to_device(PVector graphicsFrame){
  return graphicsFrame.set(-graphicsFrame.x, graphicsFrame.y);
}
/* end helper functions section ****************************************************************************************/
