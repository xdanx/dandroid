// Return a random number sampled uniformly from the range 0 to max
float sampleUniform(float max)
{
	unsigned int negposRng = 0;
	negposRng = rand();//%65536;
	// Converts random int to correct float range
	return max * (float)(negposRng) / 65536.0;
}


// Return a random number sampled from a Gaussian distribution with
// mean = mean, standard deviation = sigma
float sampleGaussian(float mean, float sigma)
{
  float u     = sampleUniform(1.0);
  float theta = sampleUniform(2 * 3.14);

  // Fix to avoid infinity problem
  if (u == 0) {
    u = 0.0001;
  }

  float r = sqrt(-2*log(u));

  float x = r * cos(theta);

  return mean + sigma * x;
}
