/*
 * player.c
 *
 *  Created on: Feb 19, 2023
 *      Author: lefucjusz
 */
#define DR_MP3_IMPLEMENTATION
#define DR_MP3_ONLY_MP3
#define DR_MP3_NO_STDIO

#include "dr_mp3.h"
#include "player.h"
#include "CS43L22.h"
#include <errno.h>

#define PLAYER_BUFFER_SIZE_FRAMES (PLAYER_BUFFER_SIZE_SAMPLES / PLAYER_CHANNELS_NUM)

typedef enum {
	BUFFER_REQ_NONE,
	BUFFER_REQ_FIRST_HALF,
	BUFFER_REQ_SECOND_HALF
} player_buffer_req_t;

typedef struct {
	int16_t dma_buffer[PLAYER_BUFFER_SIZE_SAMPLES];
	drmp3 mp3;
	volatile player_buffer_req_t buffer_req;
	player_state_t state;
	I2S_HandleTypeDef *i2s;
	I2C_HandleTypeDef *i2c;
	lfs_t *lfs;
	lfs_file_t fd;
} player_ctx_t;

static player_ctx_t ctx;

static bool is_extension(const char *filename, const char *ext) {
    const char *dot_ptr = strrchr(filename, '.');
    return ((dot_ptr != NULL) && (strcasecmp(dot_ptr, ext) == 0));
}

static bool configure_i2s(uint32_t sample_rate) {
	/* Only 44k1 and 48k supported for now */
	if ((sample_rate != I2S_AUDIOFREQ_44K) && (sample_rate != I2S_AUDIOFREQ_48K)) {
		return false;
	}

	/* Deinit I2S module */
	if (HAL_I2S_DeInit(ctx.i2s) != HAL_OK) {
		return false;
	}

	/* Set sample rate */
	ctx.i2s->Init.AudioFreq = sample_rate;

	/* Reinitialize I2S module with new setting */
	return (HAL_I2S_Init(ctx.i2s) == HAL_OK);
}

static size_t on_read(void* pUserData, void* pBufferOut, size_t bytesToRead)
{
	return lfs_file_read(ctx.lfs, &ctx.fd, pBufferOut, bytesToRead);
}

static drmp3_bool32 on_seek(void* pUserData, int offset, drmp3_seek_origin origin)
{
	return (lfs_file_seek(ctx.lfs, &ctx.fd, offset, origin == drmp3_seek_origin_start ? LFS_SEEK_SET : LFS_SEEK_CUR) >= 0);
}

void player_init(I2S_HandleTypeDef *i2s, I2C_HandleTypeDef *i2c, lfs_t *lfs) {
	ctx.buffer_req = BUFFER_REQ_NONE;
	ctx.state = PLAYER_STOPPED;
	ctx.i2s = i2s;
	ctx.i2c = i2c;
	ctx.lfs = lfs;
}

int player_start(const char *path) {
	/* Sanity check */
	if (path == NULL) {
		return -EINVAL;
	}

	/* Stop playback if not stopped */
	if (ctx.state != PLAYER_STOPPED) {
		player_stop();
	}

	/* Check if supported extension */
	if (!is_extension(path, ".mp3")) {
		return -ENOTSUP;
	}

	int err = lfs_file_open(ctx.lfs, &ctx.fd, path, LFS_O_RDONLY);
	if (err) {
		return -ENOENT;
	}

	/* Initialize decoder */
	const drmp3_bool32 drmp3_ret = drmp3_init(&ctx.mp3, on_read, on_seek, NULL, NULL);
	if (drmp3_ret != DRMP3_TRUE) {
		lfs_file_close(ctx.lfs, &ctx.fd);
		return -EIO;
	}

	/* Set proper sample rate */
	const uint32_t pcm_sample_rate = player_get_pcm_sample_rate();
	const bool i2s_ret = configure_i2s(pcm_sample_rate);
	if (!i2s_ret) {
		lfs_file_close(ctx.lfs, &ctx.fd);
		drmp3_uninit(&ctx.mp3);
		return -EINVAL;
	}

	/* Fill buffer with frames */
	const drmp3_uint64 frames_read = drmp3_read_pcm_frames_s16(&ctx.mp3, PLAYER_BUFFER_SIZE_FRAMES, ctx.dma_buffer);
	if (frames_read == 0) {
		lfs_file_close(ctx.lfs, &ctx.fd);
		drmp3_uninit(&ctx.mp3);
		return -EIO;
	}

	/* Start DMA */
	const HAL_StatusTypeDef dma_ret = HAL_I2S_Transmit_DMA(ctx.i2s, (uint16_t *)ctx.dma_buffer, PLAYER_BUFFER_SIZE_SAMPLES);
	if (dma_ret != HAL_OK) {
		lfs_file_close(ctx.lfs, &ctx.fd);
		drmp3_uninit(&ctx.mp3);
		return -EBUSY;
	}

	/* Initialize DAC */
	const bool dac_ret = CS43L22_init(ctx.i2c);
	if (!dac_ret) {
		lfs_file_close(ctx.lfs, &ctx.fd);
		drmp3_uninit(&ctx.mp3);
		return -EBUSY;
	}

	ctx.state = PLAYER_PLAYING;
	return 0;
}

void player_pause(void) { // TODO error handling
	if (ctx.state != PLAYER_PLAYING) {
		return;
	}

	CS43L22_mute(ctx.i2c, true);
	CS43L22_power_down(ctx.i2c);
	HAL_I2S_DMAPause(ctx.i2s);
	ctx.state = PLAYER_PAUSED;
}

void player_resume(void) {
	if (ctx.state != PLAYER_PAUSED) {
		return;
	}

	HAL_I2S_DMAResume(ctx.i2s);
	CS43L22_power_up(ctx.i2c);
	CS43L22_mute(ctx.i2c, false);
	ctx.state = PLAYER_PLAYING;
}

void player_stop(void) {
	if (ctx.state == PLAYER_STOPPED) {
		return;
	}

	lfs_file_close(ctx.lfs, &ctx.fd);

	CS43L22_deinit(ctx.i2c);
	HAL_I2S_DMAStop(ctx.i2s);
	drmp3_uninit(&ctx.mp3);

	ctx.buffer_req = BUFFER_REQ_NONE;
	ctx.state = PLAYER_STOPPED;
}

bool player_set_volume(int8_t volume) {
	return CS43L22_set_volume(ctx.i2c, volume);
}

player_state_t player_get_state(void) {
	return ctx.state;
}

uint64_t player_get_frames_played(void) {
	return ctx.mp3.currentPCMFrame;
}

uint32_t player_get_pcm_sample_rate(void) {
	return ctx.mp3.sampleRate;
}

uint32_t player_get_mp3_frame_bitrate(void) {
	return ctx.mp3.mp3FrameBitrate;
}

void player_task(void) {
	if ((ctx.state != PLAYER_PLAYING) || (ctx.buffer_req == BUFFER_REQ_NONE)) {
		return;
	}

	drmp3_uint64 frames_read = 0;

	switch (ctx.buffer_req) {
		case BUFFER_REQ_FIRST_HALF:
			frames_read = drmp3_read_pcm_frames_s16(&ctx.mp3, PLAYER_BUFFER_SIZE_FRAMES / 2, ctx.dma_buffer);
			ctx.buffer_req = BUFFER_REQ_NONE;
			break;

		case BUFFER_REQ_SECOND_HALF:
			frames_read = drmp3_read_pcm_frames_s16(&ctx.mp3, PLAYER_BUFFER_SIZE_FRAMES / 2, &ctx.dma_buffer[PLAYER_BUFFER_SIZE_SAMPLES / 2]);
			ctx.buffer_req = BUFFER_REQ_NONE;
			break;

		default:
			break;
	}

	if (frames_read == 0) {
		player_stop();
	}
}

/* DMA transfer interrupt handlers */
void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef *hi2s)
{
	ctx.buffer_req = BUFFER_REQ_FIRST_HALF;
}

void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s)
{
	ctx.buffer_req = BUFFER_REQ_SECOND_HALF;
}
