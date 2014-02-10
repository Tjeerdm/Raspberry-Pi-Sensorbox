// Modified from:
//
// ALSA project - the C library reference
//  Main Page
//  Related Pages Modules Data Structures Files Examples / test / pcm.c

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sched.h>
#include <errno.h>
#include <getopt.h>
#include <alsa/asoundlib.h>
#include <sys/time.h>
#include <math.h>

#include "PCMSynthesiser.hpp"
#include "VarioSynthesiser.hpp"
#include "audio_vario.hpp"

const char *device = "plughw:0,0";	/* playback device */
static snd_pcm_sframes_t buffer_size;
static snd_pcm_sframes_t period_size;
static snd_output_t *output = NULL;
static short* buffer;

static int set_hwparams (snd_pcm_t * handle, snd_pcm_hw_params_t * params)
{
  unsigned buffer_time = 500000;	/* ring buffer length in us */
  unsigned period_time = 100000;	/* period time in us = 100ms */
  const unsigned rrate = 44100/4;	/* data rate */
  unsigned rate = rrate;

  snd_pcm_uframes_t size;
  int err, dir;

/* choose all parameters */
  err = snd_pcm_hw_params_any (handle, params);
  if (err < 0)
    {
      printf
	("Broken configuration for playback: no configurations available: %s\n",
	 snd_strerror (err));
      return err;
    }
/* set hardware resampling */
  err = snd_pcm_hw_params_set_rate_resample (handle, params, 1);
  if (err < 0)
    {
      printf ("Resampling setup failed for playback: %s\n",
	      snd_strerror (err));
      return err;
    }
/* set the interleaved read/write format */
  err = snd_pcm_hw_params_set_access (handle, params, SND_PCM_ACCESS_RW_INTERLEAVED);
  if (err < 0)
    {
      printf ("Access type not available for playback: %s\n",
	      snd_strerror (err));
      return err;
    }
/* set the sample format */
  err = snd_pcm_hw_params_set_format (handle, params, SND_PCM_FORMAT_S16);
  if (err < 0)
    {
      printf ("Sample format not available for playback: %s\n",
	      snd_strerror (err));
      return err;
    }
/* set the count of channels */
  snd_pcm_hw_params_set_channels (handle, params, 1);

/* set the stream rate */
  err = snd_pcm_hw_params_set_rate_near (handle, params, &rate, 0);
  if (err < 0)
    {
      printf ("Rate %iHz not available for playback: %s\n", rrate,
	      snd_strerror (err));
      return err;
    }
  if (rrate != rate)
    {
      printf ("Rate doesn't match (requested %iHz, get %iHz)\n", rrate, rate);
      return -EINVAL;
    }
/* set the buffer time */
  err = snd_pcm_hw_params_set_buffer_time_near (handle, params, &buffer_time, &dir);
  if (err < 0)
    {
      printf ("Unable to set buffer time %i for playback: %s\n", buffer_time,
	      snd_strerror (err));
      return err;
    }
  err = snd_pcm_hw_params_get_buffer_size (params, &size);
  if (err < 0)
    {
      printf ("Unable to get buffer size for playback: %s\n",
	      snd_strerror (err));
      return err;
    }
  buffer_size = size;
/* set the period time */
  err = snd_pcm_hw_params_set_period_time_near (handle, params, &period_time, &dir);
  if (err < 0)
    {
      printf ("Unable to set period time %i for playback: %s\n", period_time,
	      snd_strerror (err));
      return err;
    }
  err = snd_pcm_hw_params_get_period_size (params, &size, &dir);
  if (err < 0)
    {
      printf ("Unable to get period size for playback: %s\n",
	      snd_strerror (err));
      return err;
    }
  period_size = size;
/* write the parameters to device */
  err = snd_pcm_hw_params (handle, params);
  if (err < 0)
    {
      printf ("Unable to set hw params for playback: %s\n",
	      snd_strerror (err));
      return err;
    }
  return 0;
}

static int set_swparams (snd_pcm_t * handle, snd_pcm_sw_params_t * swparams)
{
  int err;
/* get the current swparams */
  err = snd_pcm_sw_params_current (handle, swparams);
  if (err < 0)
    {
      printf ("Unable to determine current swparams for playback: %s\n",
	      snd_strerror (err));
      return err;
    }
/* start the transfer when the buffer is almost full: */
/* (buffer_size / avail_min) * avail_min */
  err =
    snd_pcm_sw_params_set_start_threshold (handle, swparams,
					   (buffer_size / period_size) *
					   period_size);
  if (err < 0)
    {
      printf ("Unable to set start threshold mode for playback: %s\n",
	      snd_strerror (err));
      return err;
    }
/* allow the transfer when at least period_size samples can be processed */
  err = snd_pcm_sw_params_set_avail_min (handle, swparams, period_size);
  if (err < 0)
    {
      printf ("Unable to set avail min for playback: %s\n",
	      snd_strerror (err));
      return err;
    }
/* write the parameters to the playback device */
  err = snd_pcm_sw_params (handle, swparams);
  if (err < 0)
    {
      printf ("Unable to set sw params for playback: %s\n",
	      snd_strerror (err));
      return err;
    }
  return 0;
}

/*
* Underrun and suspend recovery
*/
static int xrun_recovery (snd_pcm_t * handle, int err)
{
  printf ("stream recovery\n");
  if (err == -EPIPE)
    {				/* under-run */
      err = snd_pcm_prepare (handle);
      if (err < 0)
	printf ("Can't recovery from underrun, prepare failed: %s\n",
		snd_strerror (err));
      return 0;
    }
  else if (err == -ESTRPIPE)
    {
      while ((err = snd_pcm_resume (handle)) == -EAGAIN)
	sleep (1);		/* wait until the suspend flag is released */
      if (err < 0)
	{
	  err = snd_pcm_prepare (handle);
	  if (err < 0)
	    printf ("Can't recovery from suspend, prepare failed: %s\n",
		    snd_strerror (err));
	}
      return 0;
    }
  return err;
}

#if ALSA_ASYNC
static
#endif
void alsa_fill_buffer (VarioSynthesiser* synth, snd_pcm_t * handle)
{
  while (1) {
    int err = snd_pcm_avail_update (handle);
    if (err < 0) {
      xrun_recovery (handle, err);
    } else {
      if (err >= period_size) {
        synth->Synthesise(buffer, period_size);
        snd_pcm_writei (handle, buffer, period_size);
      } else
        break;
    }
  }
}
#if ALSA_ASYNC
static void async_callback (snd_async_handler_t * ahandler)
{
  snd_pcm_t *handle = snd_async_handler_get_pcm (ahandler);
  VarioSynthesiser* synth = (VarioSynthesiser*) snd_async_handler_get_callback_private (ahandler);
  alsa_fill_buffer(synth, handle);
}
#endif

snd_pcm_t *
alsa_setup (VarioSynthesiser *synth)
{
  snd_pcm_t *handle;
  int err;
  snd_pcm_hw_params_t *hwparams;
  snd_pcm_sw_params_t *swparams;
  snd_pcm_hw_params_alloca (&hwparams);
  snd_pcm_sw_params_alloca (&swparams);

  err = snd_output_stdio_attach (&output, stdout, 0);
  if (err < 0)
    {
      printf ("Output failed: %s\n", snd_strerror (err));
      return 0;
    }
  printf ("Playback device is %s\n", device);
  if ((err = snd_pcm_open (&handle, device, SND_PCM_STREAM_PLAYBACK, 0)) < 0)
    {
      printf ("Playback open error: %s\n", snd_strerror (err));
      return 0;
    }
  if ((err = set_hwparams (handle, hwparams)) < 0)
    {
      printf ("Setting of hwparams failed: %s\n", snd_strerror (err));
      exit (EXIT_FAILURE);
    }
  if ((err = set_swparams (handle, swparams)) < 0)
    {
      printf ("Setting of swparams failed: %s\n", snd_strerror (err));
      exit (EXIT_FAILURE);
    }

  buffer = (short*)calloc(sizeof(short), period_size);

#if ALSA_ASYNC
  snd_async_handler_t * ahandler;
  err = snd_async_add_pcm_handler (&ahandler, handle, async_callback, synth);
  if (err < 0)
    {
      printf ("Unable to register async handler\n");
      exit (EXIT_FAILURE);
    }
#endif

  /* Start with 2 silent periods */
  char *silence = (char*)calloc(sizeof(short), period_size);
  err = snd_pcm_writei (handle, silence, period_size);
  err = snd_pcm_writei (handle, silence, period_size);
  free(silence);
  if (snd_pcm_state (handle) == SND_PCM_STATE_PREPARED)
    {
      err = snd_pcm_start (handle);
      if (err < 0)
	{
	  printf ("Start error: %s\n", snd_strerror (err));
	  exit (EXIT_FAILURE);
	}
    }

  return handle;
}
