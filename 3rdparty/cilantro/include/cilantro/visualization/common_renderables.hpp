#pragma once

#include <cilantro/config.hpp>

#ifdef HAVE_PANGOLIN
#include <cilantro/visualization/renderable.hpp>
#include <pangolin/pangolin.h>

namespace cilantro {

template <typename T, typename = int>
struct HasPoints : std::false_type {};

template <typename T>
struct HasPoints<T, decltype((void)T::points, 0)> : std::true_type {};

template <typename T, typename = int>
struct HasNormals : std::false_type {};

template <typename T>
struct HasNormals<T, decltype((void)T::normals, 0)> : std::true_type {};

template <typename T, typename = int>
struct HasColors : std::false_type {};

template <typename T>
struct HasColors<T, decltype((void)T::colors, 0)> : std::true_type {};

struct PointCloudGPUBufferObjects : public GPUBufferObjects {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  pangolin::GlBuffer pointBuffer;
  pangolin::GlBuffer normalBuffer;
  pangolin::GlBuffer colorBuffer;
  pangolin::GlBuffer normalEndPointBuffer;
};

class PointCloudRenderable : public Renderable {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef PointCloudGPUBufferObjects GPUBuffers;

  inline PointCloudRenderable(const ConstVectorSetMatrixMap<float, 3>& points,
                              const RenderingProperties& rp = RenderingProperties())
      : Renderable(rp), points(points), normals(NULL), colors(NULL), values(NULL) {
    if (points.cols() > 0) centroid = points.rowwise().mean();
  }

  template <class CloudT,
            class = typename std::enable_if<HasPoints<CloudT>::value && HasNormals<CloudT>::value &&
                                            HasColors<CloudT>::value>::type>
  inline PointCloudRenderable(const CloudT& cloud,
                              const RenderingProperties& rp = RenderingProperties())
      : Renderable(rp),
        points(cloud.points),
        normals(cloud.normals),
        colors(cloud.colors),
        values(NULL) {
    if (points.cols() > 0) centroid = points.rowwise().mean();
  }

  inline PointCloudRenderable& setPointNormals(const ConstVectorSetMatrixMap<float, 3>& normals) {
    if (normals.cols() == points.cols())
      new (&this->normals) ConstVectorSetMatrixMap<float, 3>(normals);
    buffersUpToDate = false;
    return *this;
  }

  inline PointCloudRenderable& setPointColors(const ConstVectorSetMatrixMap<float, 3>& colors) {
    if (colors.cols() == points.cols())
      new (&this->colors) ConstVectorSetMatrixMap<float, 3>(colors);
    buffersUpToDate = false;
    return *this;
  }

  inline PointCloudRenderable& setPointValues(const ConstVectorSetMatrixMap<float, 1>& values) {
    if (values.cols() == points.cols())
      new (&this->values) ConstVectorSetMatrixMap<float, 1>(values);
    buffersUpToDate = false;
    return *this;
  }

  void updateGPUBuffers(GPUBufferObjects& gl_objects);

  void render(GPUBufferObjects& gl_objects);

private:
  ConstVectorSetMatrixMap<float, 3> points;
  ConstVectorSetMatrixMap<float, 3> normals;
  ConstVectorSetMatrixMap<float, 3> colors;
  ConstVectorSetMatrixMap<float, 1> values;
};

struct PointCorrespondencesGPUBufferObjects : public GPUBufferObjects {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  pangolin::GlBuffer lineEndPointBuffer;
};

class PointCorrespondencesRenderable : public Renderable {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef PointCloudGPUBufferObjects GPUBuffers;

  inline PointCorrespondencesRenderable(const ConstVectorSetMatrixMap<float, 3>& dst_points,
                                        const ConstVectorSetMatrixMap<float, 3>& src_points,
                                        const RenderingProperties& rp = RenderingProperties())
      : Renderable(rp), dstPoints(dst_points), srcPoints(src_points) {
    if (srcPoints.cols() == dstPoints.cols() && srcPoints.cols() > 0) {
      centroid = 0.5f * (srcPoints + dstPoints).rowwise().mean();
    }
  }

  template <class CorrT>
  inline PointCorrespondencesRenderable(const ConstVectorSetMatrixMap<float, 3>& dst_points,
                                        const ConstVectorSetMatrixMap<float, 3>& src_points,
                                        const CorrT& correspondences,
                                        const RenderingProperties& rp = RenderingProperties())
      : Renderable(rp) {
    if (!correspondences.empty()) {
      dstPoints.resize(3, correspondences.size());
      srcPoints.resize(3, correspondences.size());
      Eigen::Vector3f sum(Eigen::Vector3f::Zero());
      for (size_t i = 0; i < correspondences.size(); i++) {
        dstPoints.col(i) = dst_points.col(correspondences[i].indexInFirst);
        srcPoints.col(i) = src_points.col(correspondences[i].indexInSecond);
        sum += dstPoints.col(i) + srcPoints.col(i);
      }
      centroid = sum * (0.5f / (correspondences.size()));
    }
  }

  template <class CloudT, class = typename std::enable_if<HasPoints<CloudT>::value>::type>
  inline PointCorrespondencesRenderable(const CloudT& dst_cloud, const CloudT& src_cloud,
                                        const RenderingProperties& rp = RenderingProperties())
      : Renderable(rp), dstPoints(dst_cloud.points), srcPoints(src_cloud.points) {
    if (srcPoints.cols() == dstPoints.cols() && srcPoints.cols() > 0) {
      centroid = 0.5f * (srcPoints + dstPoints).rowwise().mean();
    }
  }

  template <class CloudT, class CorrT,
            class = typename std::enable_if<HasPoints<CloudT>::value>::type>
  PointCorrespondencesRenderable(const CloudT& dst_cloud, const CloudT& src_cloud,
                                 const CorrT& correspondences,
                                 const RenderingProperties& rp = RenderingProperties())
      : Renderable(rp) {
    if (!correspondences.empty()) {
      dstPoints.resize(3, correspondences.size());
      srcPoints.resize(3, correspondences.size());
      Eigen::Vector3f sum(Eigen::Vector3f::Zero());
      for (size_t i = 0; i < correspondences.size(); i++) {
        dstPoints.col(i) = dst_cloud.points.col(correspondences[i].indexInFirst);
        srcPoints.col(i) = src_cloud.points.col(correspondences[i].indexInSecond);
        sum += dstPoints.col(i) + srcPoints.col(i);
      }
      centroid = sum * (0.5f / (correspondences.size()));
    }
  }

  void updateGPUBuffers(GPUBufferObjects& gl_objects);

  void render(GPUBufferObjects& gl_objects);

private:
  VectorSet<float, 3> dstPoints;
  VectorSet<float, 3> srcPoints;
};

class CoordinateFrameRenderable : public Renderable {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef GPUBufferObjects GPUBuffers;

  inline CoordinateFrameRenderable(const Eigen::Matrix4f& tf = Eigen::Matrix4f::Identity(),
                                   float scale = 1.0f,
                                   const RenderingProperties& rp = RenderingProperties())
      : Renderable(rp, tf.topRightCorner(3, 1)), transform(tf), scale(scale) {}

  void updateGPUBuffers(GPUBufferObjects& gl_objects);

  void render(GPUBufferObjects& gl_objects);

private:
  Eigen::Matrix4f transform;
  float scale;
};

class CameraFrustumRenderable : public Renderable {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef GPUBufferObjects GPUBuffers;

  inline CameraFrustumRenderable(size_t width, size_t height, const Eigen::Matrix3f& intrinsics,
                                 const Eigen::Matrix4f& pose = Eigen::Matrix4f::Identity(),
                                 float scale = 1.0f,
                                 const RenderingProperties& rp = RenderingProperties())
      : Renderable(rp, pose.topRightCorner(3, 1)),
        width(width),
        height(height),
        inverseIntrinsics(intrinsics.inverse()),
        pose(pose),
        scale(scale) {}

  void updateGPUBuffers(GPUBufferObjects& gl_objects);

  void render(GPUBufferObjects& gl_objects);

private:
  size_t width;
  size_t height;
  Eigen::Matrix3f inverseIntrinsics;
  Eigen::Matrix4f pose;
  float scale;
};

struct TriangleMeshGPUBufferObjects : public GPUBufferObjects {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  pangolin::GlBuffer vertexBuffer;
  pangolin::GlBuffer normalBuffer;
  pangolin::GlBuffer colorBuffer;
  pangolin::GlBuffer normalEndPointBuffer;
};

class TriangleMeshRenderable : public Renderable {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef TriangleMeshGPUBufferObjects GPUBuffers;

  inline TriangleMeshRenderable(const ConstVectorSetMatrixMap<float, 3>& vertices,
                                const std::vector<std::vector<size_t>>& faces,
                                const RenderingProperties& rp = RenderingProperties())
      : Renderable(rp),
        vertices(vertices),
        faces(faces),
        vertexNormals(NULL),
        faceNormals(NULL),
        vertexColors(NULL),
        faceColors(NULL),
        vertexValues(NULL),
        faceValues(NULL) {
    if (vertices.cols() > 0) centroid = vertices.rowwise().mean();
    // initFaceNormals();
  }

  template <class CloudT,
            class = typename std::enable_if<HasPoints<CloudT>::value && HasNormals<CloudT>::value &&
                                            HasColors<CloudT>::value>::type>
  inline TriangleMeshRenderable(const CloudT& vertex_cloud,
                                const std::vector<std::vector<size_t>>& faces,
                                const RenderingProperties& rp = RenderingProperties())
      : Renderable(rp),
        vertices(vertex_cloud.points),
        faces(faces),
        vertexNormals(NULL),
        faceNormals(NULL),
        vertexColors(NULL),
        faceColors(NULL),
        vertexValues(NULL),
        faceValues(NULL) {
    if (vertex_cloud.points.cols() > 0) centroid = vertex_cloud.points.rowwise().mean();
    if (vertex_cloud.normals.cols() == vertex_cloud.points.cols())
      vertexNormals = vertex_cloud.normals;
    if (vertex_cloud.colors.cols() == vertex_cloud.points.cols())
      vertexColors = vertex_cloud.colors;
    // initFaceNormals();
  }

  inline TriangleMeshRenderable& setVertexNormals(
      const ConstVectorSetMatrixMap<float, 3>& vertex_normals) {
    if (vertex_normals.cols() == vertices.cols())
      new (&this->vertexNormals) ConstVectorSetMatrixMap<float, 3>(vertex_normals);
    buffersUpToDate = false;
    return *this;
  }

  inline TriangleMeshRenderable& setFaceNormals(
      const ConstVectorSetMatrixMap<float, 3>& face_normals) {
    if (face_normals.cols() == faces.size())
      new (&this->faceNormals) ConstVectorSetMatrixMap<float, 3>(face_normals);
    buffersUpToDate = false;
    return *this;
  }

  inline TriangleMeshRenderable& setVertexColors(
      const ConstVectorSetMatrixMap<float, 3>& vertex_colors) {
    if (vertex_colors.cols() == vertices.cols())
      new (&this->vertexColors) ConstVectorSetMatrixMap<float, 3>(vertex_colors);
    buffersUpToDate = false;
    return *this;
  }

  inline TriangleMeshRenderable& setFaceColors(
      const ConstVectorSetMatrixMap<float, 3>& face_colors) {
    if (face_colors.cols() == faces.size())
      new (&this->faceColors) ConstVectorSetMatrixMap<float, 3>(face_colors);
    buffersUpToDate = false;
    return *this;
  }

  inline TriangleMeshRenderable& setVertexValues(
      const ConstVectorSetMatrixMap<float, 1>& vertex_values) {
    if (vertex_values.cols() == vertices.cols())
      new (&this->vertexValues) ConstVectorSetMatrixMap<float, 1>(vertex_values);
    buffersUpToDate = false;
    return *this;
  }

  inline TriangleMeshRenderable& setFaceValues(
      const ConstVectorSetMatrixMap<float, 1>& face_values) {
    if (face_values.cols() == faces.size())
      new (&this->faceValues) ConstVectorSetMatrixMap<float, 1>(face_values);
    buffersUpToDate = false;
    return *this;
  }

  void updateGPUBuffers(GPUBufferObjects& gl_objects);

  void render(GPUBufferObjects& gl_objects);

private:
  ConstVectorSetMatrixMap<float, 3> vertices;
  const std::vector<std::vector<size_t>>& faces;
  ConstVectorSetMatrixMap<float, 3> vertexNormals;
  ConstVectorSetMatrixMap<float, 3> faceNormals;
  ConstVectorSetMatrixMap<float, 3> vertexColors;
  ConstVectorSetMatrixMap<float, 3> faceColors;
  ConstVectorSetMatrixMap<float, 1> vertexValues;
  ConstVectorSetMatrixMap<float, 1> faceValues;

  VectorSet<float, 3> computedFaceNormals;
  void initFaceNormals();
};

struct TextGPUBufferObjects : public GPUBufferObjects {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  std::shared_ptr<pangolin::GlFont> glFont;
  pangolin::GlText glText;
};

class TextRenderable : public Renderable {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef TextGPUBufferObjects GPUBuffers;

  inline TextRenderable(const std::string& text, const Eigen::Ref<const Eigen::Vector3f>& position,
                        const RenderingProperties& rp = RenderingProperties())
      : Renderable(rp, position), text(text) {
    drawLast = true;
  }

  inline TextRenderable(const std::string& text, float pos_x, float pos_y, float pos_z,
                        const RenderingProperties& rp = RenderingProperties())
      : Renderable(rp, Eigen::Vector3f(pos_x, pos_y, pos_z)), text(text) {
    drawLast = true;
  }

  void updateGPUBuffers(GPUBufferObjects& gl_objects);

  void render(GPUBufferObjects& gl_objects);

private:
  std::string text;
};

}  // namespace cilantro
#endif
