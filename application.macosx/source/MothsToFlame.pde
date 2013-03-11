// The Nature of Code
// Daniel Shiffman
// http://natureofcode.com

// Demonstration of Craig Reynolds' "Flocking" behavior
// See: http://www.red3d.com/cwr/
// Rules: Cohesion, Separation, Alignment

// Click mouse to add boids into the system

Flock flock;

PImage lightImage;
PVector brightestAreaPos;
String imageName = "flower.jpg";

void setup() {
   // Make a new flow field with "resolution" of 16
   lightImage = loadImage(imageName);
     size(lightImage.width, lightImage.height);
       lightImage.loadPixels();
  brightestAreaPos = findBrightestAreaInImage();
  flock = new Flock();
  // Add an initial set of boids into the system
  for (int i = 0; i < 200; i++) {
    Boid b = new Boid(width/2,height/2);
    flock.addBoid(b);
  }
  smooth();
  

}

void draw() {
   image(lightImage, 0, 0);
  flock.run();
  
    // Instructions
  fill(0);
  //text("Drag the mouse to generate new boids.",10,height-16);
}

// Add a new boid into the System
void mouseDragged() {
  flock.addBoid(new Boid(mouseX,mouseY));
}

PVector findBrightestAreaInImage(){
  int imageDimension = lightImage.height * lightImage.width;
  float brightestPixelValue = 0;
  float pixelBrightnessValue ;
  int brightestIndex = 0;
  //lets find the brightest pixel in the image
  for(int i = 0; i < imageDimension; i++)
  {
    color c = lightImage.pixels[i]; 
    pixelBrightnessValue = (red(c) + green(c) + blue(c))/3.0;
    if(pixelBrightnessValue > brightestPixelValue)
    {
      brightestPixelValue = pixelBrightnessValue;
      brightestIndex = i;
    }
    
  }
  int y =brightestIndex/lightImage.width;
  int x = brightestIndex % lightImage.width;
  print("X:  " + String.valueOf(x) + "    Y:    " + String.valueOf(y));
  return new PVector(x, y);
}

