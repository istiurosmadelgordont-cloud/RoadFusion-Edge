#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <gst/allocators/gstdmabuf.h>
#include <gst/video/video.h>
#include <rga/rga.h>
#include <rga/im2d.h>
#include <cstdlib>
#include <iostream>
#include <chrono>

int main(int argc, char** argv) {
  gst_init(&argc, &argv);
  if (argc < 2) return 1;
  GError* error = nullptr;
  GstElement* pipeline = gst_parse_launch(argv[1], &error);
  if (error) { std::cerr << error->message << std::endl; return 2; }
  GstElement* sink = gst_bin_get_by_name(GST_BIN(pipeline), "out");
  gst_element_set_state(pipeline, GST_STATE_PLAYING);
  int frames = 0, dma_frames = 0;
  auto start = std::chrono::steady_clock::now();
  while (true) {
    GstSample* sample = gst_app_sink_try_pull_sample(GST_APP_SINK(sink), 5 * GST_SECOND);
    if (!sample) break;
    GstBuffer* buffer = gst_sample_get_buffer(sample);
    bool dma = false;
    for (guint i = 0; i < gst_buffer_n_memory(buffer); ++i) {
      GstMemory* memory = gst_buffer_peek_memory(buffer, i);
      dma |= gst_is_dmabuf_memory(memory);
      if (frames == 0) std::cout << "memory=" << i << " allocator="
          << memory->allocator->mem_type << " dma=" << gst_is_dmabuf_memory(memory)
          << " fd=" << (gst_is_dmabuf_memory(memory) ? gst_dmabuf_memory_get_fd(memory) : -1) << '\n';
    }
    if (frames == 0) {
      gchar* caps = gst_caps_to_string(gst_sample_get_caps(sample));
      std::cout << caps << " bytes=" << gst_buffer_get_size(buffer) << '\n';
      g_free(caps);
      GstVideoMeta* meta = gst_buffer_get_video_meta(buffer);
      if (meta) std::cout << "stride=" << meta->stride[0] << ',' << meta->stride[1]
                          << " offset=" << meta->offset[0] << ',' << meta->offset[1] << '\n';
      if (dma && meta) {
        void* output = nullptr;
        posix_memalign(&output, 64, 640 * 360 * 3);
        GstMemory* memory = gst_buffer_peek_memory(buffer, 0);
        const int fd = gst_dmabuf_memory_get_fd(memory);
        const int hstride = static_cast<int>(meta->offset[1] / meta->stride[0]);
        const rga_buffer_t source = wrapbuffer_fd_t(fd, 640, 360,
                                                    meta->stride[0], hstride,
                                                    RK_FORMAT_YCbCr_420_SP);
        const rga_buffer_t target = wrapbuffer_virtualaddr_t(output, 640, 360,
                                                             640, 360,
                                                             RK_FORMAT_BGR_888);
        const IM_STATUS status = imcvtcolor(source, target,
                                            RK_FORMAT_YCbCr_420_SP,
                                            RK_FORMAT_BGR_888,
                                            IM_YUV_TO_RGB_BT601_LIMIT);
        std::cout << "rga_status=" << status << " hstride=" << hstride
                  << " message=" << imStrError(status) << '\n';
        free(output);
      }
    }
    ++frames;
    dma_frames += dma;
    gst_sample_unref(sample);
  }
  double seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
  std::cout << "frames=" << frames << " dma_frames=" << dma_frames << " seconds=" << seconds
            << " fps=" << frames / seconds << std::endl;
  GstBus* bus = gst_element_get_bus(pipeline);
  GstMessage* message = gst_bus_pop_filtered(bus, GST_MESSAGE_ERROR);
  if (message) { gchar* debug = nullptr; gst_message_parse_error(message, &error, &debug);
    std::cerr << error->message << " " << (debug ? debug : "") << std::endl; gst_message_unref(message); }
  gst_object_unref(bus);
  gst_element_set_state(pipeline, GST_STATE_NULL);
  gst_object_unref(sink);
  gst_object_unref(pipeline);
  return frames == 605 && dma_frames == frames ? 0 : 3;
}
