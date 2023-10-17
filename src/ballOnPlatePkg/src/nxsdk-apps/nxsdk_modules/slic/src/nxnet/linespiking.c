/*
INTEL CORPORATION CONFIDENTIAL AND PROPRIETARY

Copyright © 2017-2021 Intel Corporation.

This software and the related documents are Intel copyrighted
materials, and your use of them is governed by the express
license under which they were provided to you (License). Unless
the License provides otherwise, you may not use, modify, copy,
publish, distribute, disclose or transmit  this software or the
related documents without Intel's prior written permission.

This software and the related documents are provided as is, with
no express or implied warranties, other than those that are
expressly stated in the License.
*/

#include "linespiking.h"

int numAxon = 2032;
int coreId = 4;
int timestepPerImage = 32;
int bytesPerImage = 50;
int firstTimestep = 1;
int spikeInterval = 8;
int trainingPhase = 1;
int xImgSize =20;
int yImgSize =20;
int imageChannelId;
int imageData[50];
uint8_t dvsSpike[2032];
int numLines[2];
ScanLine lines[508];
int numscanlines =508;
int numsegments =4;

int do_spiking(runState *s){
    if (s->time_step == 1){
		imageChannelId = getChannelID("nximage");
        GenScanLines(lines,numLines);
    }
    if(s->time_step > spikeInterval && s->time_step%spikeInterval==firstTimestep)
        return 1;
    else
        return 0;
}

void GetSpikingAxonsForImage()
{
int img[400];
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
    GenDVSSpike(dvsSpike, img, lines, numLines[0], numscanlines, numsegments);
}

void InjectModelSpikes(runState *RunState)
{
   if (RunState->time_step%timestepPerImage==spikeInterval+firstTimestep) {
        readChannel(imageChannelId, imageData, bytesPerImage);
        unsigned int *tmp_ptr = (unsigned int *)&dvsSpike[0];
        for(int i=0; i < numscanlines; i++) tmp_ptr[i] = 0;
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

void run_spiking(runState *RunState){
    InjectModelSpikes(RunState);
}

#define FIX(x, d) ((x)>=0 ? (x)>>(d) : -((-(x))>>(d)))

void GenScanLines(ScanLine *lines, int *numLines)
{
    // generate scan lines
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

void GenDVSSpike(uint8_t *dvsspike, int *img1, ScanLine *lines, int numLines1, int numScanLines, int numSegments)
{
    /*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %       Code below Generates Spikes running on Lakemont by advancing scanline
    %
    % Memory to store image: 400 bits (original images) + 100 bits (scaled
    %   images) = 500 bits per image
    % Memory to store input axon spikes: 2000 * 1 bit per axons = 2000 bits
    % dvsspike can be derived from input axon spikes together with delay index
    % otherwise it can be stored in time sequences
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/


    // We use both original image and its scaled version of half size
    // scale image to half size
int img2[100];
    int tmp;
	int xImg2 =10;
	int yImg2 =10;

    for (int i=0; i<xImg2; i++){
        for (int j=0; j<yImg2; j++){
            tmp = img1[(2*i)*xImgSize + 2*j] + img1[(2*i)*xImgSize + 2*j+1] + img1[(2*i+1)*xImgSize + 2*j] + img1[(2*i+1)*xImgSize + 2*j+1];
            if (tmp>=2)
                img2[i*xImg2 + j]=1;
            else
                img2[i*xImg2 + j]=0;
        }
    }


    // generate spikes
    int *img;
    int lineIdx1, lineIdx2;
    int n, n1;
    int i1, i2, j1, j2, j;
    int m;
    int anchor;
    int im;
    int pf;
    int idx;
    int ct, th_pf;

    for (int scale=0; scale<2; scale++){
        if (scale==0){
            img = img1;
            lineIdx1 = 0;
            lineIdx2 = numLines1;
            n=xImgSize-1;
            n1=xImgSize;
        }
        else{ //scale==1
            img = img2;
            lineIdx1 = numLines1;
            lineIdx2 = numScanLines;
            n=xImg2-1;
            n1=xImg2;
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
                        else pf = numSegments*(i-i1)/(i2-i1);
                        idx = k + numScanLines*pf;
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
                        else pf = numSegments*(i1-i)/(i1-i2);
                        idx = k + numScanLines*pf;
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
                            pf = numSegments*(j-j1)/(j2-j1);
                            idx = k + numScanLines*pf;
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
                            pf = numSegments*(j1-j)/(j1-j2);
                            idx = k + numScanLines*pf;
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

