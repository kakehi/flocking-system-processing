import controlP5.*;
Boids Boids;

// Declare GUIs
ControlP5 jControl;
Slider sliderRanCoh, sliderRanSep, sliderRanAli, sliderMagCoh, sliderMagSep, sliderMagAli, sliderMaxA, sliderMaxV;

float Range_Cohesion, Range_Separation, Range_Alignment;
float Magnitude_Cohesion, Magnitude_Separation, Magnitude_Alignment;
float boidCount = 300;

float randV = 6.0f; // the force each tries to go random
float Max_Acceleration, Max_Velocity;

float speedMultiplier;

float radius = 10;


void setup() {
  size(800, 600);

  // Create a boids world
  Boids = new Boids();

  // Max acceleration and speed
  Max_Acceleration = 5.0f;
  Max_Velocity = 3.0f;

  // Magnitudes
  Magnitude_Cohesion = .5f;
  Magnitude_Separation = 4f;
  Magnitude_Alignment = 4f;

  // Range
  Range_Cohesion = 40;
  Range_Separation = 10;
  Range_Alignment = 40;
  
  // Initiate set of agencies into Boids
  for (int i = 0; i < boidCount; i++) {
    Boids.addBoid(new Boid());
  }

  createGUI();
}

void draw() {
  background(50);

  Boids.update();
}

void createGUI() {

  // Create GUI
  jControl = new ControlP5(this);

  Slider sliderRanCoh = jControl.addSlider(
    "Range_Cohesion", 
    0 /* min value */, 
    100 /* max value */, 
    40 /* current value */, 
    10 /* pos X */, 
    10 /* posY */, 
    200 /* width */, 
    20 /* height */
  );
  
  Slider sliderRanSep = jControl.addSlider("Range_Separation", 0, 100, 10, 10, 40, 200, 20);
  Slider sliderRanAli = jControl.addSlider("Range_Alignment", 0, 100, 40, 10, 70, 200, 20);
  Slider sliderMagCoh = jControl.addSlider("Magnitude_Cohesion", 0, 10, .5, 320, 10, 200, 20);
  Slider sliderMagSep = jControl.addSlider("Magnitude_Separation", 0, 10, 4, 320, 40, 200, 20);
  Slider sliderMagAli = jControl.addSlider("Magnitude_Alignment", 0, 10, 4, 320, 70, 200, 20);
  
  Slider sliderMaxA = jControl.addSlider("Max_Acceleration", 0, 10, 5, 10, 120, 200, 20);
  Slider sliderMaxV = jControl.addSlider("Max_Velocity", 0, 10, 3, 10, 150, 200, 20);
  
}


class Boids {
  ArrayList<Boid> boids; // Declare a list of boids

  Boids() {
    boids = new ArrayList<Boid>(); // Initialize the boids list
  }

  void update() {
    for (Boid b : boids) {
      b.update(boids);  // Passing the entire list of boids to each boid individually
    }
  }


  void addBoid(Boid b) {
    boids.add(b);
  }
}



class Boid {

  PVector pos, a, v, coh, sep, ali;

  Boid () {  
    // randomize position
    //pos = new PVector(width/2, height/2);
    pos = new PVector(random(0, width), random(0, height));

    a = new PVector();
    v = new PVector(random(-3, 3), random(-3, 3));
  } 


  void update(ArrayList<Boid> boids) {

    // update vector
    coh  = PVector.mult(cohesion(
      getNeighbors(boids, Range_Cohesion)
      ), Magnitude_Cohesion);
    sep  = PVector.mult(separation(
      getNeighbors(boids, Range_Separation)
      ), Magnitude_Separation);
    ali  = PVector.mult(alignment(
      getNeighbors(boids, Range_Alignment)
      ), Magnitude_Alignment);
    
    // combine accelerations
    PVector randomizeV = new PVector(random(-1 * randV, randV), random(-1 * randV, randV));
    a = PVector.add(coh, PVector.add(sep, ali));
    a = PVector.add(a, randomizeV);
    // clamp accelerations
    a.set(clampV(a.x, Max_Acceleration), clampV(a.y, Max_Acceleration));
    
    // get vectors
    v = PVector.add(a, v);
    // clamp velocity
    v.set(clampV(v.x, Max_Velocity), clampV(v.y, Max_Velocity));

    // update position
    pos = PVector.add(pos, v);

    // make sure boid stays in the boundary
    pos.set(
      wrapAroundFloat(pos.x, 0, width), 
      wrapAroundFloat(pos.y, 0, height)
      );

    // draw a boid
    ellipse(pos.x, pos.y, radius, radius);
  }



  // Get the neighborhood list
  /* */
  ArrayList<Boid> getNeighbors(ArrayList<Boid> boids, float radius) {
    ArrayList<Boid> neighbors;
    neighbors = new ArrayList<Boid>();

    for (Boid other : boids) {
      if (other != this) {
        if (PVector.dist(this.pos, other.pos) <= radius) {
          neighbors.add(other);
        }
      }
    }

    return neighbors;
  }


  float wrapAroundFloat(float value, float min, float max) {

    if (value > max)
      value = min;
    else if (value < min)
      value  = max;
    return value;
  }



  // Clamp speed or acceleration to max
  float clampV(float r, float max) {
    if (r > max) {
      return max;
    } else if (r < (-1 * max)) {
      return -1 * max;
    }
    return r;
  }

  PVector cohesion(ArrayList<Boid> neighbors) {

    PVector r = new PVector();

    // if there are no neighbors, finish here and return 0 vector
    if (neighbors.size() == 0) {
      return r;
    }

    // find the center position of this boid’s neighbors,
    for (Boid other : neighbors) {
      r = PVector.add(r, other.pos);
    }
    r = PVector.div(r, neighbors.size());

    // find a new vector from this boid’s position to this center position
    r = PVector.sub(r, this.pos);

    // return the norma
    return r.normalize();
  }



  PVector separation(ArrayList<Boid> neighbors) {

    PVector r = new PVector();

    // if there are no neighbors, finish here and return 0 vector
    if (neighbors.size() == 0) {
      return r;
    }

    // add the contribution of each neighbor towards me
    for (Boid other : neighbors) {

      PVector towardsMe = new PVector();
      towardsMe = PVector.sub(this.pos, other.pos);

      // force contribution will vary inversely proportional
      if (towardsMe.mag() > 0) {

        // to distance or even the square of the distance
        r = PVector.add(r, PVector.div(towardsMe.normalize(), towardsMe.mag()));
      }
    }

    return r.normalize();
  }



  PVector alignment(ArrayList<Boid> neighbors) {

    PVector r = new PVector();

    // if there are no neighbors, finish here and return 0 vector.
    if (neighbors.size() == 0) {
      return r;
    }

    // for each of its neighbor boid, get its current vector and combine all to find the average vector within its neighbors 
    for (Boid boid : neighbors) {
      r = PVector.add(r, boid.v);
    }

    return r.normalize();
  }
}
