#include<iostream>

using namespace std;

int main(int argc, char *argv[])
{
    int n = 10;

    /*int x[n], y[n];
    for(int i = 0; i < n; i++)
    {
         x[i] = atoi(argv[i+1].c_str());
         y[i] = atoi(argv[i+1+n].c_str());
    }*/
/*
Pos = [(3.1, 0.4),
              (2.8, -0.7),
             (4.1, 0),
             (2, 1.4),
             (3.4, 1.2),
             (4.1, 0),
             (2.4, 0.6),
             (4.5, 1.2),
             (2.6, 0.5),
             (1.8, 0.6)
*/
    float x[] = {3.1, 2.8, 4.1, 2, 3.4, 4.1, 2.4, 4.5, 2.6, 1.8};
    float y[] = {0.4, -0.7, 0, 1.4, 1.2, 0, 0.6, 1.2, 0.5, 0.6};

    float sx = 0;
    float sy = 0;

    for(int i = 0; i < n; i++)
    {
    	sx += x[i];
    	sy += y[i];
    }
    float xm = sx / n;
    float ym = sy / n;

    float p[4];

    float s1 = 0, s2 = 0, s3 = 0;
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

    
    scanf("%d",p[0]);
    return 0;
    
}
