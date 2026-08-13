#include "top_view/top_view.hpp"

#include <QApplication>
#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QFormLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QImage>
#include <QLabel>
#include <QLineEdit>
#include <QMainWindow>
#include <QMessageBox>
#include <QMouseEvent>
#include <QPixmap>
#include <QPushButton>
#include <QScrollArea>
#include <QSplitter>
#include <QStatusBar>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>

#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <nlohmann/json.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <stdexcept>
#include <string>
#include <unordered_map>

namespace fs = std::filesystem;

namespace {

class ImageLabel final : public QLabel {
public:
  explicit ImageLabel(QWidget *parent = nullptr) : QLabel(parent) {
    setMouseTracking(true);
    setAlignment(Qt::AlignCenter);
    setStyleSheet("QLabel { background: #202124; color: #dddddd; }");
  }
  std::function<void(const QPoint &)> mouse_handler;

protected:
  void mouseMoveEvent(QMouseEvent *event) override {
    if (mouse_handler)
      mouse_handler(event->position().toPoint());
    QLabel::mouseMoveEvent(event);
  }
};

QPixmap pixmapFromBgr(const cv::Mat &bgr) {
  cv::Mat rgb;
  if (bgr.channels() == 3)
    cv::cvtColor(bgr, rgb, cv::COLOR_BGR2RGB);
  else if (bgr.channels() == 4)
    cv::cvtColor(bgr, rgb, cv::COLOR_BGRA2RGBA);
  else
    cv::cvtColor(bgr, rgb, cv::COLOR_GRAY2RGB);
  const QImage::Format format =
      rgb.channels() == 4 ? QImage::Format_RGBA8888 : QImage::Format_RGB888;
  return QPixmap::fromImage(
      QImage(rgb.data, rgb.cols, rgb.rows, static_cast<int>(rgb.step), format)
          .copy());
}

class TopViewWindow final : public QMainWindow {
public:
  TopViewWindow() {
    setWindowTitle("Calibration Top-View — Automatic / Manual Common GUI");
    resize(1500, 900);

    auto *central = new QWidget(this);
    auto *root = new QVBoxLayout(central);
    auto *inputs = new QGroupBox("입력 및 기준평면", central);
    auto *grid = new QGridLayout(inputs);
    int row = 0;
    addPathRow(grid, row++, "Camera image", image_path_,
               "Images (*.png *.jpg *.jpeg *.bmp *.tif *.tiff)", false);
    addPathRow(grid, row++, "Camera intrinsic JSON", camera_path_,
               "JSON (*.json)", false);
    addPathRow(grid, row++, "Calibration RT JSON", rt_path_, "JSON (*.json)",
               false);
    addPathRow(grid, row++, "Child→plane RT JSON", plane_path_, "JSON (*.json)",
               true);

    grid->addWidget(new QLabel("RT object key"), row, 0);
    rt_key_ = new QComboBox;
    rt_key_->setEditable(true);
    rt_key_->addItem("자동 감지", "");
    rt_key_->addItem("Automatic estimated", "estimated");
    rt_key_->addItem("Manual extrinsic", "extrinsic");
    rt_key_->addItem("Manual camera-lidar", "manual_camera_lidar");
    rt_key_->addItem("Automatic camera-lidar", "automatic_camera_lidar");
    rt_key_->addItem("Generic transform", "transform");
    grid->addWidget(rt_key_, row, 1, 1, 2);
    ++row;

    plane_warning_ = new QLabel("Plane RT가 없으면 calibration RT의 child "
                                "frame Z=0을 임시 기준평면으로 "
                                "사용합니다. 실제 지면 Top-View에는 측정된 "
                                "T_child_ground가 필요합니다.");
    plane_warning_->setWordWrap(true);
    plane_warning_->setStyleSheet("QLabel { color: #b35c00; }");
    grid->addWidget(plane_warning_, row++, 0, 1, 3);

    auto *settings = new QGroupBox("Top-View 범위", central);
    auto *form = new QFormLayout(settings);
    x_min_ = makeSpin(-1000.0, 1000.0, -5.0, 0.1, " m");
    x_max_ = makeSpin(-1000.0, 1000.0, 5.0, 0.1, " m");
    y_min_ = makeSpin(-1000.0, 1000.0, 0.0, 0.1, " m");
    y_max_ = makeSpin(-1000.0, 1000.0, 10.0, 0.1, " m");
    pixels_per_meter_ = makeSpin(1.0, 1000.0, 80.0, 5.0, " px/m");
    grid_spacing_ = makeSpin(0.01, 100.0, 1.0, 0.25, " m");
    form->addRow("X min / max", pairWidget(x_min_, x_max_));
    form->addRow("Y min / max", pairWidget(y_min_, y_max_));
    form->addRow("Resolution", pixels_per_meter_);
    form->addRow("Grid spacing", grid_spacing_);
    draw_grid_ = new QCheckBox("Grid와 원점 축 표시");
    draw_grid_->setChecked(true);
    form->addRow(draw_grid_);
    scale_intrinsic_ =
        new QCheckBox("영상 해상도가 다르면 intrinsic을 비례 스케일");
    scale_intrinsic_->setChecked(false);
    form->addRow(scale_intrinsic_);

    auto *top_controls = new QSplitter(Qt::Horizontal, central);
    top_controls->addWidget(inputs);
    top_controls->addWidget(settings);
    top_controls->setStretchFactor(0, 3);
    top_controls->setStretchFactor(1, 2);
    root->addWidget(top_controls);

    auto *buttons = new QHBoxLayout;
    auto *render = new QPushButton("Load && Render Top-View");
    save_ = new QPushButton("Save PNG + metadata");
    save_->setEnabled(false);
    buttons->addWidget(render);
    buttons->addWidget(save_);
    buttons->addStretch();
    mode_label_ = new QLabel("Mode: not loaded");
    buttons->addWidget(mode_label_);
    root->addLayout(buttons);

    original_label_ = new ImageLabel;
    original_label_->setText("Camera image");
    top_view_label_ = new ImageLabel;
    top_view_label_->setText("Top-View");
    auto *original_scroll = makeScroll(original_label_, "Original camera");
    auto *top_scroll = makeScroll(top_view_label_, "Rectified top-view");
    auto *views = new QSplitter(Qt::Horizontal, central);
    views->addWidget(original_scroll);
    views->addWidget(top_scroll);
    views->setStretchFactor(0, 1);
    views->setStretchFactor(1, 1);
    root->addWidget(views, 1);

    setCentralWidget(central);
    statusBar()->showMessage("Camera, intrinsic, RT를 선택하세요.");
    connect(render, &QPushButton::clicked, this, [this] { renderView(); });
    connect(save_, &QPushButton::clicked, this, [this] { saveResult(); });
    top_view_label_->mouse_handler = [this](const QPoint &point) {
      if (rendered_top_view_.empty())
        return;
      const double x =
          active_config_.x_min_m + point.x() / active_config_.pixels_per_meter;
      const double y =
          active_config_.y_max_m - point.y() / active_config_.pixels_per_meter;
      statusBar()->showMessage(QString("Plane coordinate: X=%1 m, Y=%2 m")
                                   .arg(x, 0, 'f', 3)
                                   .arg(y, 0, 'f', 3));
    };
  }

  void setInitialPaths(const QString &image, const QString &camera,
                       const QString &rt, const QString &plane) {
    image_path_->setText(image);
    camera_path_->setText(camera);
    rt_path_->setText(rt);
    plane_path_->setText(plane);
  }

  void renderIfReady() {
    if (!image_path_->text().isEmpty() && !camera_path_->text().isEmpty() &&
        !rt_path_->text().isEmpty())
      renderView();
  }

private:
  QLineEdit *image_path_ = nullptr;
  QLineEdit *camera_path_ = nullptr;
  QLineEdit *rt_path_ = nullptr;
  QLineEdit *plane_path_ = nullptr;
  QComboBox *rt_key_ = nullptr;
  QDoubleSpinBox *x_min_ = nullptr;
  QDoubleSpinBox *x_max_ = nullptr;
  QDoubleSpinBox *y_min_ = nullptr;
  QDoubleSpinBox *y_max_ = nullptr;
  QDoubleSpinBox *pixels_per_meter_ = nullptr;
  QDoubleSpinBox *grid_spacing_ = nullptr;
  QCheckBox *draw_grid_ = nullptr;
  QCheckBox *scale_intrinsic_ = nullptr;
  QLabel *plane_warning_ = nullptr;
  QLabel *mode_label_ = nullptr;
  ImageLabel *original_label_ = nullptr;
  ImageLabel *top_view_label_ = nullptr;
  QPushButton *save_ = nullptr;
  cv::Mat source_image_;
  cv::Mat rendered_top_view_;
  nlohmann::json metadata_;
  top_view::TopViewConfig active_config_;

  static QDoubleSpinBox *makeSpin(double minimum, double maximum, double value,
                                  double step, const QString &suffix) {
    auto *spin = new QDoubleSpinBox;
    spin->setRange(minimum, maximum);
    spin->setDecimals(3);
    spin->setValue(value);
    spin->setSingleStep(step);
    spin->setSuffix(suffix);
    return spin;
  }

  static QWidget *pairWidget(QWidget *first, QWidget *second) {
    auto *widget = new QWidget;
    auto *layout = new QHBoxLayout(widget);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->addWidget(first);
    layout->addWidget(second);
    return widget;
  }

  static QScrollArea *makeScroll(QWidget *content, const QString &name) {
    auto *scroll = new QScrollArea;
    scroll->setWidget(content);
    scroll->setWidgetResizable(false);
    scroll->setAlignment(Qt::AlignCenter);
    scroll->setObjectName(name);
    return scroll;
  }

  void addPathRow(QGridLayout *layout, int row, const QString &label,
                  QLineEdit *&line, const QString &filter, bool optional) {
    layout->addWidget(new QLabel(label + (optional ? " (optional)" : "")), row,
                      0);
    line = new QLineEdit;
    layout->addWidget(line, row, 1);
    auto *button = new QPushButton("Browse");
    layout->addWidget(button, row, 2);
    connect(button, &QPushButton::clicked, this, [this, line, label, filter] {
      const QString selected = QFileDialog::getOpenFileName(
          this, "Select " + label, line->text(), filter);
      if (!selected.isEmpty())
        line->setText(selected);
    });
  }

  QString selectedKey() const {
    if (rt_key_->currentIndex() == 0)
      return {};
    const QVariant data = rt_key_->currentData();
    return data.isValid() ? data.toString() : rt_key_->currentText();
  }

  top_view::TopViewConfig configFromUi() const {
    top_view::TopViewConfig config;
    config.x_min_m = x_min_->value();
    config.x_max_m = x_max_->value();
    config.y_min_m = y_min_->value();
    config.y_max_m = y_max_->value();
    config.pixels_per_meter = pixels_per_meter_->value();
    config.grid_spacing_m = grid_spacing_->value();
    config.draw_grid = draw_grid_->isChecked();
    return config;
  }

  static std::string inferMode(const nlohmann::json &document,
                               const top_view::Transform &transform) {
    const std::string method = document.value("method", "");
    if (method.find("charuco") != std::string::npos ||
        transform.child_frame.find("marker") != std::string::npos)
      return "MANUAL";
    if (method.find("automatic") != std::string::npos ||
        transform.child_frame.find("lidar") != std::string::npos)
      return "AUTOMATIC";
    return "GENERIC";
  }

  void renderView() {
    try {
      if (image_path_->text().isEmpty() || camera_path_->text().isEmpty() ||
          rt_path_->text().isEmpty())
        throw std::runtime_error(
            "Camera image, intrinsic JSON and RT JSON are required");
      source_image_ =
          cv::imread(image_path_->text().toStdString(), cv::IMREAD_COLOR);
      if (source_image_.empty())
        throw std::runtime_error("Cannot read camera image");
      const auto camera_document =
          top_view::readJson(camera_path_->text().toStdString());
      auto camera = top_view::parseCameraModel(camera_document);
      if (camera.width != source_image_.cols ||
          camera.height != source_image_.rows) {
        if (!scale_intrinsic_->isChecked())
          throw std::runtime_error(
              "Image resolution differs from intrinsic profile. Enable "
              "intrinsic scaling only when the image is a pure resize.");
        const double sx =
            static_cast<double>(source_image_.cols) / camera.width;
        const double sy =
            static_cast<double>(source_image_.rows) / camera.height;
        camera.k(0, 0) *= sx;
        camera.k(0, 2) *= sx;
        camera.k(1, 1) *= sy;
        camera.k(1, 2) *= sy;
        camera.width = source_image_.cols;
        camera.height = source_image_.rows;
      }

      const auto rt_document =
          top_view::readJson(rt_path_->text().toStdString());
      const auto input_transform =
          top_view::parseTransform(rt_document, selectedKey().toStdString(),
                                   "camera_optical", "reference_frame");
      top_view::Transform plane_transform;
      bool assumed_plane = plane_path_->text().isEmpty();
      if (assumed_plane)
        plane_transform =
            top_view::identityPlaneTransform(input_transform.child_frame);
      else
        plane_transform = top_view::parseTransform(
            top_view::readJson(plane_path_->text().toStdString()), {},
            input_transform.child_frame, "ground_plane");
      const auto camera_plane =
          top_view::compose(input_transform, plane_transform);
      active_config_ = configFromUi();
      rendered_top_view_ = top_view::renderTopView(
          source_image_, camera, camera_plane, active_config_);
      metadata_ =
          top_view::renderMetadata(camera, input_transform, plane_transform,
                                   camera_plane, active_config_);
      metadata_["source"] = {{"image", image_path_->text().toStdString()},
                             {"camera", camera_path_->text().toStdString()},
                             {"calibration_rt", rt_path_->text().toStdString()},
                             {"plane_rt", plane_path_->text().toStdString()}};
      metadata_["plane_assumed_from_rt_child"] = assumed_plane;
      metadata_["calibration_mode"] = inferMode(rt_document, input_transform);

      original_label_->setPixmap(pixmapFromBgr(source_image_));
      original_label_->resize(original_label_->pixmap().size());
      top_view_label_->setPixmap(pixmapFromBgr(rendered_top_view_));
      top_view_label_->resize(top_view_label_->pixmap().size());
      mode_label_->setText(
          QString("Mode: %1 | %2 → %3")
              .arg(QString::fromStdString(
                  metadata_["calibration_mode"].get<std::string>()))
              .arg(QString::fromStdString(input_transform.parent_frame))
              .arg(QString::fromStdString(camera_plane.child_frame)));
      plane_warning_->setStyleSheet(
          assumed_plane ? "QLabel { color: #b35c00; font-weight: bold; }"
                        : "QLabel { color: #146b2e; }");
      save_->setEnabled(true);
      statusBar()->showMessage(
          assumed_plane ? "Rendered with assumed child-frame Z=0 plane; not a "
                          "surveyed ground plane."
                        : "Rendered with explicit child-to-plane transform.");
    } catch (const std::exception &error) {
      save_->setEnabled(false);
      QMessageBox::critical(this, "Top-View error", error.what());
      statusBar()->showMessage("Render failed");
    }
  }

  void saveResult() {
    if (rendered_top_view_.empty())
      return;
    const QString selected = QFileDialog::getSaveFileName(
        this, "Save Top-View", "top_view.png", "PNG image (*.png)");
    if (selected.isEmpty())
      return;
    try {
      if (!cv::imwrite(selected.toStdString(), rendered_top_view_))
        throw std::runtime_error("Failed to save Top-View PNG");
      fs::path metadata_path(selected.toStdString());
      metadata_path.replace_extension(".top_view.json");
      std::ofstream output(metadata_path);
      if (!output)
        throw std::runtime_error("Failed to save metadata JSON");
      output << std::setw(2) << metadata_ << '\n';
      statusBar()->showMessage(
          QString("Saved %1 and %2")
              .arg(selected)
              .arg(QString::fromStdString(metadata_path.string())));
    } catch (const std::exception &error) {
      QMessageBox::critical(this, "Save error", error.what());
    }
  }
};

std::unordered_map<std::string, QString> parseArguments(int argc, char **argv) {
  std::unordered_map<std::string, QString> result;
  for (int index = 1; index < argc; ++index) {
    const std::string key = argv[index];
    if ((key == "--image" || key == "--camera" || key == "--rt" ||
         key == "--plane") &&
        index + 1 < argc)
      result[key] = QString::fromLocal8Bit(argv[++index]);
  }
  return result;
}

} // namespace

int main(int argc, char **argv) {
  QApplication application(argc, argv);
  application.setApplicationName("Calibration Top-View");
  const auto args = parseArguments(argc, argv);
  TopViewWindow window;
  const auto get = [&args](const std::string &key) {
    const auto found = args.find(key);
    return found == args.end() ? QString{} : found->second;
  };
  window.setInitialPaths(get("--image"), get("--camera"), get("--rt"),
                         get("--plane"));
  window.show();
  QTimer::singleShot(0, &window, [&window] { window.renderIfReady(); });
  for (int index = 1; index < argc; ++index)
    if (std::string(argv[index]) == "--smoke-test")
      QTimer::singleShot(100, &application, &QApplication::quit);
  return application.exec();
}
