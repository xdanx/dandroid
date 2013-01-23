#include<iostream>

using namespace std;

int main(int argc, char *argv[])
{
    int n = 2;

    /*int x[n], y[n];
    for(int i = 0; i < n; i++)
    {
         x[i] = atoi(argv[i+1].c_str());
         y[i] = atoi(argv[i+1+n].c_str());
    }*/

    int x[] = {1, 1};
    int y[] = {1, 1};

    int sx = 0;
    int sy = 0;

    for(int i = 0; i < n; i++)
    {
    	sx += x[i];
    	sy += y[i];
    }
    float xm = sx / n;
    float ym = sy / n;

    float p[4];

    int s1 = 0, s2 = 0, s3 = 0;
    for(int i = 0; i < n; i++)
    {
    	s1 += (x[i] - xm) * (x[i] - xm);
    	s2 += (x[i] - xm) * (y[i] - ym);
    	s3 += (y[i] - ym) * (y[i] - ym);
    }

    p[0] = s1 / n;
    p[1] = s2 / n;
    p[2] = s2 / n;
    p[3] = s3 / n;

    for(int i = 0; i < 4; i++)
    {
        cout << "p[" << i << "] = " << p[i] << endl;
    }

    return 0;
}
