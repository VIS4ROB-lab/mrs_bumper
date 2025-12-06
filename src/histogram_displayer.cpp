// clang: MatousFormat
#include <mrs_lib/subscriber_handler.h>

#include <mrs_msgs/msg/histogram.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>

cv::Point cursor_pos;
void mouse_callback([[maybe_unused]] int event, int x, int y,
                    [[maybe_unused]] int flags,
                    [[maybe_unused]] void* userdata) {
  cursor_pos = cv::Point(x, y);
}

/* draw_hist() function //{ */
void draw_hist(const std::vector<float>& hist, cv::Mat& hist_img,
               unsigned highlight_first = 0) {
  double max_val = 0;
  const auto bins = hist.size();
  cv::minMaxLoc(hist, 0, &max_val, 0, 0);
  const unsigned height = hist_img.rows;
  const float hscale = height / max_val;
  const float vscale = hist_img.cols / bins;
  for (unsigned b = 0; b < bins; b++) {
    const auto bin_val = hist.at(b);
    cv::Scalar color = cv::Scalar::all(255);
    if (b < highlight_first) color = cv::Scalar(255, 0, 0);
    cv::rectangle(hist_img, cv::Point(b * vscale, height),
                  cv::Point(b * vscale + vscale, height - bin_val * hscale),
                  color, -1);
  }
}
//}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("histogram_displayer");
  RCLCPP_INFO(node->get_logger(), "Node initialized.");

  /** Create publishers and subscribers //{**/
  // Initialize other subs and pubs
  mrs_lib::SubscriberHandlerOptions shopts(node);
  auto sh_hist = mrs_lib::SubscriberHandler<mrs_msgs::msg::Histogram>(
      shopts, "histogram", rclcpp::Duration(5, 0));

  //}

  /* Open OpenCV windows to display the debug info //{ */
  int window_flags =
      cv::WINDOW_NORMAL | cv::WINDOW_KEEPRATIO | cv::WINDOW_GUI_NORMAL;
  std::string winname = "histogram";
  cv::namedWindow(winname, window_flags);
  cv::setMouseCallback(winname, mouse_callback, NULL);
  //}

  rclcpp::Rate r(30);
  const int bot_rows = 100;
  const int right_cols = 100;

  while (rclcpp::ok()) {
    rclcpp::spin_some(node);

    if (sh_hist.newMsg()) {
      const mrs_msgs::msg::Histogram hist_msg = *(sh_hist.getMsg());
      const int hr = 1000;
      const int hc = 1000;
      cv::Mat disp_im = cv::Mat::zeros(hr + bot_rows, hc + right_cols, CV_8UC3);
      cv::Mat hist_roi = disp_im(cv::Rect(cv::Point(0, 0), cv::Size(hr, hc)));
      draw_hist(hist_msg.bins, hist_roi, hist_msg.bin_mark);
      /* hist_roi(cv::Rect(cv::Point(vscale*hist_msg.bin_mark, 0),
       * cv::Point(vscale*hist_msg.bin_mark+1, hc-1))) = cv::Scalar(255, 0, 0);
       */
      cv::imshow(winname, disp_im);
      cv::waitKey(1);
    }

    r.sleep();
  }
  rclcpp::shutdown();
  return 0;
}
