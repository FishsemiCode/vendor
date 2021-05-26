/****************************************************************************
 * apps/external/example/codec2test/codec2_test.c
 *
 *
 *   Copyright (C) 2019 Fishsemi Inc. All rights reserved.
 *   Author: qiaohaijiao<qiaohaijiao@fishsemi.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <string.h>

#include <codec2_wrapper.h>

#define PROFILE 0

#define ONLY_DECODE 0
#define ONLY_ENCODE 1
#define ENC_DEC     2

#define SAMPLES_PER_FRAME 160

#define BITS_SIZE_3200 64
#define BITS_SIZE_2400 48

static void help(void)
{
  printf("-d: decode only \n");
  printf("-e: encode only \n");
  printf("-m: mode 0: 3200, 1: 2400\n");
  printf("-i: input file \n");
  printf("-o: output file \n");
  printf("-p: other output file, when enc first, then dec.\n");
  printf("Example: \n");
  printf("c2test -d -m 0 -i 1.bin -o 1.pcm \n");
  printf("c2test -e -m 0 -i 1.pcm -o 1.bin \n");
  printf("c2test -m 0 -i 1.pcm -o 1.bin -p 1_1.pcm\n");
}

/****************************************************************************
 * Public Funtions
 ****************************************************************************/
#ifdef CONFIG_BUILD_KERNEL
int main(int argc, char *argv[])
#else
int c2test_main(int argc, char *argv[])
#endif
{
  int            mode;
  void          *codec2;
  char          *in, *out, *out_p;
  FILE          *fin, *fout, *fout_2;

  short         *buf, *buf_out;
  unsigned char *bits;
  int            nsam, nbit, nbyte;
  int            option, ret;
  int            test = ENC_DEC;

#if PROFILE
  struct timespec start, end, delta;
  uint32_t total, frames;
  total = frames = 0;
#endif

  while ((option = getopt(argc, argv, "edm:i:o:p:h")) != -1)
  {
    switch (option)
      {
        case 'e':
          test = ONLY_ENCODE;
          break;
        case 'd':
          test = ONLY_DECODE;
          break;
        case 'm':
          mode = atoi(optarg);
          break;
        case 'i':
          in = optarg;
          break;
        case 'o':
          out = optarg;
          break;
        case 'p':
          out_p = optarg;
          break;
        case 'h':
          help();
          return 0;
        default:
          printf("ERROR ARGS\n");
          help();
          return -1;
      }
  }


  if (mode == CODEC2_MODE_3200)
    {
      nbit = BITS_SIZE_3200;
    }
  else if (mode == CODEC2_MODE_2400)
    {
      nbit = BITS_SIZE_2400;
    }
  else
   {
     printf("Error in mode: %d.  Must be 0, 1\n", mode);
     return -1;
   }

  if ((fin = fopen(in,"rb")) == NULL)
    {
      printf("Error opening input file: %s: %s.\n", in, strerror(errno));
      goto cleanup;
    }

  if ((fout = fopen(out,"wb")) == NULL)
    {
      printf("Error opening output file: %s: %s.\n", out, strerror(errno));
      goto cleanup;
    }

  /* init codec handler */
  codec2 = codec2_wrapper_init(mode);
  if (codec2 == NULL)
    {
      printf("Error create codec handler.\n");
      goto cleanup;
    }


  /* init pcm buffer, 20ms */
  nsam   = SAMPLES_PER_FRAME;
  buf    = (short *)malloc(nsam * sizeof(short));
  if (buf == NULL)
    {
      printf("Error malloc buffer %d.\n", __LINE__);
      goto cleanup;
    }

  /* init raw buffer */
  nbyte  = (nbit + 7) / 8;
  bits = (unsigned char *)malloc(nbyte * sizeof(char));
  if (bits == NULL)
    {
      printf("Error malloc buffer %d.\n", __LINE__);
      goto cleanup;
    }


  if (test == ENC_DEC)
    {
      buf_out = (short *)malloc(nsam * sizeof(short));
      if (buf_out == NULL)
        {
          printf("Error malloc buffer %d.\n", __LINE__);
          goto cleanup;
        }

      if ((fout_2 = fopen(out_p, "wb")) == NULL)
        {
          printf("Error opening output file: %s: %s.\n", out_p, strerror(errno));
          goto cleanup;
        }
    }
  else
    {
      buf_out = NULL;
      fout_2  = NULL;
    }

  switch (test)
    {
      case ONLY_ENCODE:
        {
          while(fread(buf, sizeof(short), nsam, fin) == (size_t)nsam)
            {
              #if PROFILE
              frames++;
              clock_gettime(CLOCK_MONOTONIC, &start);
              #endif

              codec2_wrapper_enc(codec2, buf, bits);

              #if PROFILE
              clock_gettime(CLOCK_MONOTONIC, &end);
              clock_timespec_subtract(&end, &start, &delta);
              total += (delta.tv_sec * 1000000 + delta.tv_nsec / 1000);
              #endif
              ret = fwrite(bits, sizeof(char), nbyte, fout);
              if (ret != nbyte)
                {
                  printf("Error write out bin file: %d %s.\n", __LINE__, strerror(errno));
                  break;
                }
            }
          #if PROFILE
          printf("enc average %d (%d %d)\n", total / frames, total, frames);
          #endif
          break;
        }
      case ONLY_DECODE:
        {
          while (fread(bits, sizeof(char), nbyte, fin) == (size_t)nbyte)
            {
              #if PROFILE
              frames++;
              clock_gettime(CLOCK_MONOTONIC, &start);
              #endif

              codec2_wrapper_dec(codec2, bits, buf);
              #if PROFILE
              clock_gettime(CLOCK_MONOTONIC, &end);
              clock_timespec_subtract(&end, &start, &delta);
              total += (delta.tv_sec * 1000000 + delta.tv_nsec / 1000);
              #endif
              ret = fwrite(buf, sizeof(short), nsam, fout);
              if (ret != nsam)
                {
                  printf("Error write out pcm file: %d %s.\n", __LINE__, strerror(errno));
                  break;
                }
            }

          #if PROFILE
          printf("dec average %d (%d %d)\n", total/frames, total, frames);
          #endif
          break;
        }

        case ENC_DEC:
          {
            while(fread(buf, sizeof(short), nsam, fin) == (size_t)nsam)
              {
                codec2_wrapper_enc(codec2, buf, bits);
                ret = fwrite(bits, sizeof(char), nbyte, fout);
                if (ret != nbyte)
                  {
                    printf("Error write out bin file: %d %s.\n", __LINE__, strerror(errno));
                    break;
                  }

                codec2_wrapper_dec(codec2, bits, buf_out);
                ret = fwrite(buf_out, sizeof(short), nsam, fout_2);
                if (ret != nsam)
                  {
                    printf("Error write out pcm file: %d %s.\n", __LINE__, strerror(errno));
                    break;
                  }
              }

             break;
          }
        default:
          break;
      }

cleanup:
  if (codec2 != NULL)
    codec2_wrapper_exit(codec2);

  if (buf != NULL)
    free(buf);

  if (buf_out != NULL)
    free(buf_out);

  if (bits != NULL)
    free(bits);

  if (fin != NULL)
    fclose(fin);

  if (fout != NULL)
    fclose(fout);

  if (fout_2 != NULL)
    fclose(fout_2);

  return 0;
}

