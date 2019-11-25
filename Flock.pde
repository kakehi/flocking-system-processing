Boids Boids;

float Rc, Rs, Ra;
float Kc, Ks, Ka;
float boidCount = 300;

float randV = 6.0f; // the force each tries to go random
float maxA, maxV;

float speedMultiplier;

float radius = 10;

void setup() {
  size(800, 600);
  
  // Create a boids world
  Boids = new Boids();
  
  // Magnitudes
  Kc = .5f;
  Ks = 4f;
  Ka = 4f;
  
  // Range
  Rc = 40;
  Rs = 10;
  Ra = 40;
  
  // Acceleration
  maxA = 5.0f;
  
  // Speed
  maxV = 3.0f;
  
  // Initiate set of agencies into Boids
  for (int i = 0; i < boidCount; i++) {
    Boids.addBoid(new Boid());
  }
}

void draw() {
  background(50);
  
  Boids.update();
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


  void addBoid(Boid b){
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
       getNeighbors(boids, Rc)
     ), Kc);
    sep  = PVector.mult(separation(
      getNeighbors(boids, Rs)
    ), Ks);
    ali  = PVector.mult(alignment(
      getNeighbors(boids, Ra)
    ), Ka);
    // combine accelerations
    PVector randomizeV = new PVector(random(-1 * randV, randV), random(-1 * randV, randV));
    a = PVector.add(coh, PVector.add(sep, ali));
    a = PVector.add(a, randomizeV);
    // clamp accelerations
    a.set(clampV(a.x, maxA), clampV(a.y, maxA));
    // get vectors
    //PVector randomizeV = new PVector(random(-1 * randV, randV), random(-1 * randV, randV));
    // add the random vector factor
    //v = PVector.add(a, PVector.add(v, randomizeV));
    v = PVector.add(a, v);
    // clamp velocity
    v.set(clampV(v.x, maxV), clampV(v.y, maxV));
    
    // update position
    pos = PVector.add(pos, v);
    
    // make sure boid is in the boundary
    pos.set(
       wrapAroundFloat(pos.x, 0, width),
       wrapAroundFloat(pos.y, 0, height)
    );
    
    // draw a boid
    ellipse(pos.x, pos.y, radius, radius);
  }
  
  
  
  // Get the neighborhood list
  /* */
  ArrayList<Boid> getNeighbors(ArrayList<Boid> boids, float radius){
     ArrayList<Boid> neighbors;
     neighbors = new ArrayList<Boid>();
     
     for(Boid other : boids) {
        if(other != this){
          if(PVector.dist(this.pos, other.pos) <= radius){
                  neighbors.add(other);
          }
        }
      }
      
     return neighbors;
  }


  float wrapAroundFloat(float value, float min, float max){

    if(value > max)
      value = min;
    else if(value < min)
      value  = max;
    return value;
  }
  
  
  
  // Clamp speed or acceleration to max
  float clampV(float r, float max){
    if(r > max){
      return max;
    } else if(r < (-1 * max)){
      return -1 * max;
    }
    return r;
  }

  PVector cohesion(ArrayList<Boid> neighbors){
    
    PVector r = new PVector();
    
    // if there are no neighbors, finish here and return 0 vector
    if(neighbors.size() == 0){
      return r;
    }
    
    // find the center position of this boid’s neighbors,
    for(Boid other : neighbors) {
      r = PVector.add(r, other.pos);
    }
    r = PVector.div(r, neighbors.size());
    
    // find a new vector from this boid’s position to this center position
    r = PVector.sub(r, this.pos);
    
    // return the norma
    return r.normalize();
  }
  
  
  
  PVector separation(ArrayList<Boid> neighbors){
    
    PVector r = new PVector();
    
    // if there are no neighbors, finish here and return 0 vector
    if(neighbors.size() == 0){
      return r;
    }
    
    // add the contribution of each neighbor towards me
    for(Boid other : neighbors) {
      
      PVector towardsMe = new PVector();
      towardsMe = PVector.sub(this.pos, other.pos);
      
      // force contribution will vary inversely proportional
      if(towardsMe.mag() > 0){

         // to distance or even the square of the distance
         r = PVector.add(r, PVector.div(towardsMe.normalize(), towardsMe.mag()));
      }
    }
    
    return r.normalize();
  }
  
  
  
  PVector alignment(ArrayList<Boid> neighbors){
    
    PVector r = new PVector();
    
    // if there are no neighbors, finish here and return 0 vector.
    if(neighbors.size() == 0){
      return r;
    }
    
    // for each of its neighbor boid, get its current vector and combine all to find the average vector within its neighbors 
    for(Boid boid : neighbors) {
      r = PVector.add(r, boid.v);
    }
    
    return r.normalize();
  }
}
