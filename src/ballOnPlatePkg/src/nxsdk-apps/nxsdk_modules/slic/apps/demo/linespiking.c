#include "linespiking.h"

// Flag to indicate training phase
int trainingPhase = 1;
// ChannelId of the training channel
int imageChannelId;
// Array to store training data
int imageData[50];
// Array to store output of linescan generator
uint8_t dvsSpike[1920];
// Number of lines
int numLines[2];
// Structure to store scanlines
ScanLine lines[480];

// Function to decide when to insert spike
int do_spiking(runState *s){
    // Get the channel id on first timestep
    if (s->time_step == 1){
        imageChannelId = getChannelID("nximage");
        GenScanLines(lines,numLines);
    }
    // Check if its time to insert spike
    if(s->time_step > spikeInterval && s->time_step%spikeInterval==firstTimestep){
        return 1;
    }
    else
        return 0;
}

// Call InjectModel Spikes to insert spikes
void run_spiking(runState *RunState){
    InjectModelSpikes(RunState);
}

// Function reads an image from channel and then generate and insert spike trains for it
void InjectModelSpikes(runState *RunState)
{
   if (RunState->time_step%timestepPerImage==spikeInterval+firstTimestep) {
        readChannel(imageChannelId, imageData, bytesPerImage);
        unsigned int *tmp_ptr = (unsigned int *)&dvsSpike[0];
        for(int i=0; i < 480; i++) tmp_ptr[i] = 0;
        GetSpikingAxonsForImage();
   }
   for(int i = 0; i < numAxon; i++) {
       if(dvsSpike[i]) {
           int axon_id =i*2;
           CoreId core;
           core.id = coreId;
           nx_send_discrete_spike(RunState->time_step,core,axon_id);
       }
   }
}

// Expands binary image of 50 bytes into array of 400 integers each having 0 or 1 value
void GetSpikingAxonsForImage(void)
{
    int img[20*20]; //binary images
    int N0=480;
    int pp=4;
    int ct, tmp;
    ct=0;
    for (int i=0; i<bytesPerImage; i++){
        tmp =  imageData[i];
#ifdef ENDIAN
        for (int j=0; j<8; j++){
            img[ct] = tmp & 1;
            tmp = tmp >> 1;
            ct++;
        }
#else
        for (int j=8; j>0; j--){
            ct = i*8+j-1;
            img[ct] = tmp & 1;
            tmp = tmp >> 1;
        }
#endif
    }
    GenDVSSpike(dvsSpike, img, lines, numLines[0], N0, pp);
}

#define FIX(x, d) ((x)>=0 ? (x)>>(d) : -((-(x))>>(d)))

// Function which generates scan lines
void GenScanLines(ScanLine *lines, int *numLines)
{
    // generate 480 scan lines
    int ct=0;
    int sz[2]={20,10};

    for (int scale=0; scale<2; scale++){
        int n=sz[scale];

        //horizontal line
        for (int j=0; j<n; j++){
            lines[ct].i1=0;
            lines[ct].j1=j;
            lines[ct].i2=n-1;
            lines[ct].j2=j;
	    lines[ct].m=0;
            ct++;

           //reverse line
           lines[ct].i1 = lines[ct-1].i2;
           lines[ct].j1 = lines[ct-1].j2;
           lines[ct].i2 = lines[ct-1].i1;
           lines[ct].j2 = lines[ct-1].j1;
	   lines[ct].m=0;
           ct++;
        }  


        //vertical line
        for (int i=0; i<n; i++){
            lines[ct].i1=i;
            lines[ct].j1=0;
            lines[ct].i2=i;
            lines[ct].j2=n-1;
            lines[ct].m=0;
            ct++;

           //reverse line
           lines[ct].i1 = lines[ct-1].i2;
           lines[ct].j1 = lines[ct-1].j2;
           lines[ct].i2 = lines[ct-1].i1;
           lines[ct].j2 = lines[ct-1].j1;
           lines[ct].m=0;
           ct++;
        }   


        // angle 45 degree or slope m=1
        //right half
        for (int i=0; i<n-3; i++){
            lines[ct].i1=i;
            lines[ct].j1=0;
            lines[ct].i2=n-1;
            lines[ct].j2 = lines[ct].i2-lines[ct].i1+lines[ct].j1;
	    lines[ct].m=2;
            ct++;

           //reverse line
           lines[ct].i1 = lines[ct-1].i2;
           lines[ct].j1 = lines[ct-1].j2;
           lines[ct].i2 = lines[ct-1].i1;
           lines[ct].j2 = lines[ct-1].j1;
           lines[ct].m=2;
           ct++;
        }   

        //left half
        for (int i=3; i<n-1; i++){
            lines[ct].i1=i;
            lines[ct].j1=n-1;
            lines[ct].i2=0;
            lines[ct].j2 = lines[ct].i2-lines[ct].i1+lines[ct].j1;
            lines[ct].m=2;
            ct++;

           //reverse line
           lines[ct].i1 = lines[ct-1].i2;
           lines[ct].j1 = lines[ct-1].j2;
           lines[ct].i2 = lines[ct-1].i1;
           lines[ct].j2 = lines[ct-1].j1;
           lines[ct].m=2;
           ct++;
        }



        // angle -45 degree or slope m=-1
        //left half
        for (int i=3; i<n-1; i++){
            lines[ct].i1=i;
            lines[ct].j1=0;
            lines[ct].i2=0;
            lines[ct].j2 = i;
            lines[ct].m=-2;
            ct++;

           //reverse line
           lines[ct].i1 = lines[ct-1].i2;
           lines[ct].j1 = lines[ct-1].j2;
           lines[ct].i2 = lines[ct-1].i1;
           lines[ct].j2 = lines[ct-1].j1;
            lines[ct].m=-2;
           ct++;
        }

        //right half
        for (int i=0; i<n-3; i++){
            lines[ct].i1=i;
            lines[ct].j1=n-1;
            lines[ct].i2=n-1;
            lines[ct].j2 = i;
            lines[ct].m=-2;
            ct++;

           //reverse line
           lines[ct].i1 = lines[ct-1].i2;
           lines[ct].j1 = lines[ct-1].j2;
           lines[ct].i2 = lines[ct-1].i1;
           lines[ct].j2 = lines[ct-1].j1;
            lines[ct].m=-2;
           ct++;
        }


        // slope m=0.5;
        int d = n/2;
        //middle region
        for (int j=0; j<d+1; j++){
            lines[ct].i1=0;
            lines[ct].j1=j;
            lines[ct].i2=n-1;
            lines[ct].j2 = n - d + j -1;
            lines[ct].m=1; 
            ct++;

           //reverse line
           lines[ct].i1 = lines[ct-1].i2;
           lines[ct].j1 = lines[ct-1].j2;
           lines[ct].i2 = lines[ct-1].i1;
           lines[ct].j2 = lines[ct-1].j1;
            lines[ct].m=1;
           ct++;
        }

        //left region
        for (int j=d+1; j<n-5; j=j+2){
            lines[ct].i1=0;
            lines[ct].j1=j;
            lines[ct].j2=n-1;
            lines[ct].i2 = 2*(lines[ct].j2-lines[ct].j1) + lines[ct].i1;
            lines[ct].m=1;
            ct++;

           //reverse line
           lines[ct].i1 = lines[ct-1].i2;
           lines[ct].j1 = lines[ct-1].j2;
           lines[ct].i2 = lines[ct-1].i1;
           lines[ct].j2 = lines[ct-1].j1;
            lines[ct].m=1;
           ct++;
        }

        //right region
        for (int j=n-d-2; j>=4; j=j-2){
            lines[ct].i2=n-1;
            lines[ct].j1=0;
            lines[ct].j2=j;
            lines[ct].i1 = -2*(lines[ct].j2-lines[ct].j1) + lines[ct].i2;
            lines[ct].m=1;
            ct++;

           //reverse line
           lines[ct].i1 = lines[ct-1].i2;
           lines[ct].j1 = lines[ct-1].j2;
           lines[ct].i2 = lines[ct-1].i1;
           lines[ct].j2 = lines[ct-1].j1;
            lines[ct].m=1;
           ct++;
        }


        //slope m=-0.5;
        //middle region
        for (int j=0; j<d+1; j++){
            lines[ct].i1=n-1;
            lines[ct].j1=j;
            lines[ct].i2=0;
            lines[ct].j2 = n - d + j -1;
            lines[ct].m=-1;
            ct++;

           //reverse line
           lines[ct].i1 = lines[ct-1].i2;
           lines[ct].j1 = lines[ct-1].j2;
           lines[ct].i2 = lines[ct-1].i1;
           lines[ct].j2 = lines[ct-1].j1;
            lines[ct].m=-1;
           ct++;
        }

        //left region
        for (int j=d+1; j<n-5; j=j+2){
            lines[ct].i1=n-1;
            lines[ct].j1=j;
            lines[ct].j2=n-1;
            lines[ct].i2 = -2*(lines[ct].j2-lines[ct].j1) + lines[ct].i1;
            lines[ct].m=-1;
            ct++;

           //reverse line
           lines[ct].i1 = lines[ct-1].i2;
           lines[ct].j1 = lines[ct-1].j2;
           lines[ct].i2 = lines[ct-1].i1;
           lines[ct].j2 = lines[ct-1].j1;
            lines[ct].m=-1;
           ct++;
        }

        //right region
        for (int j=n-d-2; j>=4; j=j-2){
            lines[ct].i2=0;
            lines[ct].j1=0;
            lines[ct].j2=j;
            lines[ct].i1 = 2*(lines[ct].j2-lines[ct].j1) + lines[ct].i2;
            lines[ct].m=-1;
            ct++;

           //reverse line
           lines[ct].i1 = lines[ct-1].i2;
           lines[ct].j1 = lines[ct-1].j2;
           lines[ct].i2 = lines[ct-1].i1;
           lines[ct].j2 = lines[ct-1].j1;
            lines[ct].m=-1;
           ct++;
        }

        // slope m=2;
        //middle region
        for (int i=0; i<d+1; i++){
            lines[ct].i1=i;
            lines[ct].j1=0;
            lines[ct].j2=n-1;
            lines[ct].i2 = n - d + i -1;
            lines[ct].m=4;
            ct++;

           //reverse line
           lines[ct].i1 = lines[ct-1].i2;
           lines[ct].j1 = lines[ct-1].j2;
           lines[ct].i2 = lines[ct-1].i1;
           lines[ct].j2 = lines[ct-1].j1;
            lines[ct].m=4;
           ct++;
        }

        //left region
        for (int i=d+1; i<n-5; i=i+2){
            lines[ct].i1=i;
            lines[ct].j1=0;
            lines[ct].i2=n-1;
            lines[ct].j2 = 2*(lines[ct].i2-lines[ct].i1) + lines[ct].j1;
            lines[ct].m=4;
            ct++;

           //reverse line
           lines[ct].i1 = lines[ct-1].i2;
           lines[ct].j1 = lines[ct-1].j2;
           lines[ct].i2 = lines[ct-1].i1;
           lines[ct].j2 = lines[ct-1].j1;
            lines[ct].m=4;
           ct++;
        }

        //right region
        for (int i=n-d-2; i>=4; i=i-2){
            lines[ct].i2=i;
            lines[ct].j2=n-1;
            lines[ct].i1=0;
            lines[ct].j1 = -2*(lines[ct].i2-lines[ct].i1) + lines[ct].j2;
            lines[ct].m=4;
            ct++;

           //reverse line
           lines[ct].i1 = lines[ct-1].i2;
           lines[ct].j1 = lines[ct-1].j2;
           lines[ct].i2 = lines[ct-1].i1;
           lines[ct].j2 = lines[ct-1].j1;
            lines[ct].m=4;
           ct++;
        }


        // slope m=-2;
        //middle region
        for (int i=0; i<d+1; i++){
            lines[ct].i1=i;
            lines[ct].j1=n-1;
            lines[ct].j2=0;
            lines[ct].i2 = n - d + i -1;
            lines[ct].m=-4; 
            ct++;

           //reverse line
           lines[ct].i1 = lines[ct-1].i2;
           lines[ct].j1 = lines[ct-1].j2;
           lines[ct].i2 = lines[ct-1].i1;
           lines[ct].j2 = lines[ct-1].j1;
            lines[ct].m=-4;
           ct++;
        }

        //left region
        for (int i=d+1; i<n-5; i=i+2){
            lines[ct].i1=i;
            lines[ct].j1=n-1;
            lines[ct].i2=n-1;
            lines[ct].j2 = -2*(lines[ct].i2-lines[ct].i1) + lines[ct].j1;
            lines[ct].m=-4;
            ct++;

           //reverse line
           lines[ct].i1 = lines[ct-1].i2;
           lines[ct].j1 = lines[ct-1].j2;
           lines[ct].i2 = lines[ct-1].i1;
           lines[ct].j2 = lines[ct-1].j1;
            lines[ct].m=-4;
           ct++;
        }

        //right region
        for (int i=n-d-2; i>=4; i=i-2){
            lines[ct].i2=i;
            lines[ct].j2=0;
            lines[ct].i1=0;
            lines[ct].j1 = 2*(lines[ct].i2-lines[ct].i1) + lines[ct].j2;
            lines[ct].m=-4;
            ct++;

           //reverse line
           lines[ct].i1 = lines[ct-1].i2;
           lines[ct].j1 = lines[ct-1].j2;
           lines[ct].i2 = lines[ct-1].i1;
           lines[ct].j2 = lines[ct-1].j1;
            lines[ct].m=-4;
           ct++;
        }
        numLines[scale]=ct; 


    }
    return;

}

// Function given Scan Lines and image, generates spike train
void GenDVSSpike(uint8_t *dvsspike, int *img1, ScanLine *lines, int numLines1, int N0, int pp)
{
    // We use both original image and its scaled version of half size
    // scale image to half size
    int img2[10*10];
    int tmp;
    //for (int i=0;i<400;i++){
    //    printf("Image2 at %d is %d\n",i,img1[i]);
    //}

    for (int i=0; i<10; i++){
        for (int j=0; j<10; j++){
            //tmp = img1[2*i][2*j] + img1[2*i][2*j+1] + img1[2*i+1][2*j] + img1[2*i+1][2*j+1];
            tmp = img1[(2*i)*20 + 2*j] + img1[(2*i)*20 + 2*j+1] + img1[(2*i+1)*20 + 2*j] + img1[(2*i+1)*20 + 2*j+1];
            if (tmp>=2)
                img2[i*10 + j]=1;
            else
                img2[i*10 + j]=0;
        }
    }


    // generate spikes
    int *img;
    int lineIdx1, lineIdx2; //9 bits
    int n, n1; //5 bits
    int i1, i2, j1, j2, j; //5 bits
    int m;
    int anchor; //1 bit
    int im; //1 bit
    int pf; //2 bits
    int idx; //11 bits

    int ct, th_pf;


    for (int scale=0; scale<2; scale++){
        if (scale==0){
            img = img1;
            lineIdx1 = 0;
            lineIdx2 = numLines1;
            n=19;
            n1=20;
        }
        else{ //scale==1
            img = img2;
            lineIdx1 = numLines1;
            lineIdx2 = N0;
            n=9;
            n1=10;
        }

        for (int k=lineIdx1; k<lineIdx2; k++){
            i1=lines[k].i1;
            i2=lines[k].i2;
            j1=lines[k].j1;
            j2=lines[k].j2;

            m = lines[k].m;

            ct=0;
            anchor=0;
            th_pf=5;
            if (i1<i2){
                for (int i=i1; i<i2; i++){
                    j = FIX(m*(i-i1),1)+j1;
                    if (j<0){
                        j=0;
                    }
                    else if (j>n){
                        j=n;
                    }
                    im = img[i*n1 + j];
                    if (!anchor && im){
                        anchor = 1;
                        if (i2-i1<th_pf) pf=0;
                        else pf = pp*(i-i1)/(i2-i1);
                        idx = k + N0*pf;
                        dvsspike[idx] = 1;

                        ct++;
                        if (ct>3) break;

                    }


                    else if (anchor && !im)
                        anchor = 0;
                }
            }
            else if (i1>i2){
                for (int i=i1; i>i2; i--){
                    j = FIX(m*(i-i1),1)+j1;
                    if (j<0){
                        j=0;
                    }
                    else if (j>n){
                        j=n;
                    }
                    im = img[i*n1 + j];
                    if (!anchor && im){
                        anchor = 1;
                        if (i1-i2<th_pf) pf=0;
                        else pf = pp*(i1-i)/(i1-i2);
                        idx = k + N0*pf;
                        dvsspike[idx] = 1;
                        ct++;
                        if (ct>3) break;
                    }
                    else if (anchor && !im)
                        anchor = 0;
                }
            }
            else{ //i1==i2
                int i=i1;
                if (j1<j2){
                    for (int j=j1; j<j2; j++){
                        im = img[i*n1 + j];
                        if (!anchor && im){
                            anchor = 1;
                            pf = pp*(j-j1)/(j2-j1);
                            idx = k + N0*pf;
                            dvsspike[idx] = 1;
                            ct++;
                            if (ct>3) break;
                        }
                        else if (anchor && !im)
                            anchor = 0;
                    }
                }
                else if (j1>j2){
                    for (j=j1; j>j2; j--){
                        im = img[i*n1 + j];
                        if (!anchor && im){
                            anchor = 1;
                            pf = pp*(j1-j)/(j1-j2);
                            idx = k + N0*pf;
                            dvsspike[idx] = 1;
                            ct++;
                            if (ct>3) break;
                        }
                        else if (anchor && !im)
                            anchor = 0;
                    }
                }
            }
        }
    }
    return;
}

