import processing.core.*; 
import processing.data.*; 
import processing.event.*; 
import processing.opengl.*; 

import java.util.HashMap; 
import java.util.ArrayList; 
import java.io.BufferedReader; 
import java.io.PrintWriter; 
import java.io.InputStream; 
import java.io.OutputStream; 
import java.io.IOException; 

public class MothsToFlame extends PApplet {

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

public void setup() {
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

public void draw() {
   image(lightImage, 0, 0);
  flock.run();
  
    // Instructions
  fill(0);
  //text("Drag the mouse to generate new boids.",10,height-16);
}

// Add a new boid into the System
public void mouseDragged() {
  flock.addBoid(new Boid(mouseX,mouseY));
}

public PVector findBrightestAreaInImage(){
  int imageDimension = lightImage.height * lightImage.width;
  float brightestPixelValue = 0;
  float pixelBrightnessValue ;
  int brightestIndex = 0;
  //lets find the brightest pixel in the image
  for(int i = 0; i < imageDimension; i++)
  {
    int c = lightImage.pixels[i]; 
    pixelBrightnessValue = (red(c) + green(c) + blue(c))/3.0f;
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

// The Nature of Code
// Daniel Shiffman
// http://natureofcode.com

// Boid class
// Methods for Separation, Cohesion, Alignment added

class Boid {

  PVector location;
  PVector velocity;
  PVector acceleration;
  PVector separate;
  PVector align;
  PVector cohese;
  PVector seek;
  PVector sum;
  float r;
  float maxforce;    // Maximum steering force
  float maxspeed;    // Maximum speed

  Boid(float x, float y) {
    acceleration = new PVector(0,0);
    velocity = new PVector(random(-1,1),random(-1,1));
    location = new PVector(x,y);
    r = 3.0f;
    maxspeed = 3;
    maxforce = 0.05f;
  }

  public void run(ArrayList<Boid> boids) {
    flock(boids);
    update();
    borders();
    render();
  }

  public void applyForce(PVector force) {
    // We could add mass here if we want A = F / M
    acceleration.add(force);
  }

  // We accumulate a new acceleration each time based on three rules
  public void flock(ArrayList<Boid> boids) {
    separate = separate(boids);   // Separation
    align = align(boids);      // Alignment
    cohese = cohesion(boids);   // Cohesion
    seek = seek(brightestAreaPos);
    // Arbitrarily weight these forces
    separate.mult(1.5f);
    align.mult(1.0f);
    cohese.mult(1.0f);
    seek.mult(1.0f);
    // Add the force vectors to acceleration
    applyForce(separate);
    applyForce(align);
    applyForce(cohese);
    applyForce(seek);
  }

  // Method to update location
  public void update() {
    // Update velocity
    velocity.add(acceleration);
    // Limit speed
    velocity.limit(maxspeed);
    location.add(velocity);
    // Reset accelertion to 0 each cycle
    acceleration.mult(0);
  }

  // A method that calculates and applies a steering force towards a target
  // STEER = DESIRED MINUS VELOCITY
  public PVector seek(PVector target) {
    PVector desired = PVector.sub(target,location);  // A vector pointing from the location to the target
    // Normalize desired and scale to maximum speed
    desired.normalize();
    desired.mult(maxspeed);
    // Steering = Desired minus Velocity
    desired.sub(velocity);
    desired.limit(maxforce);  // Limit to maximum steering force
    return desired;
  }
  
  public void render() {
    // Draw a triangle rotated in the direction of velocity
    float theta = velocity.heading2D() + radians(90);
    fill(175);
    stroke(0);
    pushMatrix();
    translate(location.x,location.y);
    rotate(theta);
    beginShape(TRIANGLES);
    vertex(0, -r*2);
    vertex(-r, r*2);
    vertex(r, r*2);
    endShape();
    popMatrix();
  }

  // Wraparound
  public void borders() {
    if (location.x < -r) location.x = width+r;
    if (location.y < -r) location.y = height+r;
    if (location.x > width+r) location.x = -r;
    if (location.y > height+r) location.y = -r;
  }

  // Separation
  // Method checks for nearby boids and steers away
  public PVector separate (ArrayList<Boid> boids) {
    float desiredseparation = 25.0f;
    PVector steer = new PVector(0,0,0);
    int count = 0;
    // For every boid in the system, check if it's too close
    for (Boid other : boids) {
      float d = PVector.dist(location,other.location);
      // If the distance is greater than 0 and less than an arbitrary amount (0 when you are yourself)
      if ((d > 0) && (d < desiredseparation)) {
        // Calculate vector pointing away from neighbor
        PVector diff = PVector.sub(location,other.location);
        diff.normalize();
        diff.div(d);        // Weight by distance
        steer.add(diff);
        count++;            // Keep track of how many
      }
    }
    // Average -- divide by how many
    if (count > 0) {
      steer.div((float)count);
    }

    // As long as the vector is greater than 0
    if (steer.magSq() > 0) {
      // Implement Reynolds: Steering = Desired - Velocity
      steer.normalize();
      steer.mult(maxspeed);
      steer.sub(velocity);
      steer.limit(maxforce);
    }
    return steer;
  }

  // Alignment
  // For every nearby boid in the system, calculate the average velocity
  public PVector align (ArrayList<Boid> boids) {
    float neighbordist = 50;
    sum = new PVector(0,0);
    int count = 0;
    for (Boid other : boids) {
      float d = PVector.dist(location,other.location);
      if ((d > 0) && (d < neighbordist)) {
        sum.add(other.velocity);
        count++;
      }
    }
    if (count > 0) {
      sum.div((float)count);
      sum.normalize();
      sum.mult(maxspeed);
      PVector steer = PVector.sub(sum,velocity);
      steer.limit(maxforce);
      return steer;
    } else {
      return new PVector(0,0);
    }
  }

  // Cohesion
  // For the average location (i.e. center) of all nearby boids, calculate steering vector towards that location
  public PVector cohesion (ArrayList<Boid> boids) {
    float neighbordist = 50;
    sum = new PVector(0,0);   // Start with empty vector to accumulate all locations
    int count = 0;
    for (Boid other : boids) {
      float d = PVector.dist(location,other.location);
      if ((d > 0) && (d < neighbordist)) {
        sum.add(other.location); // Add location
        count++;
      }
    }
    if (count > 0) {
      sum.div(count);
      return seek(sum);  // Steer towards the location
    } else {
      return new PVector(0,0);
    }
  }
}

// The Nature of Code
// Daniel Shiffman
// http://natureofcode.com

// Flock class
// Does very little, simply manages the ArrayList of all the boids
class Flock {
  ArrayList<Boid> boids; // An ArrayList for all the boids

  Flock() {
    boids = new ArrayList<Boid>(); // Initialize the ArrayList
  }

  public void run() {
    for (Boid b : boids) {
      b.run(boids);  // Passing the entire list of boids to each boid individually
    }
  }

  public void addBoid(Boid b) {
    boids.add(b);
  }

}
  static public void main(String[] passedArgs) {
    String[] appletArgs = new String[] { "MothsToFlame" };
    if (passedArgs != null) {
      PApplet.main(concat(appletArgs, passedArgs));
    } else {
      PApplet.main(appletArgs);
    }
  }
}
