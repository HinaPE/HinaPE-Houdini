// automatically generated by the FlatBuffers compiler, do not modify


#ifndef FLATBUFFERS_GENERATED_POINTSIMPLELISTSEARCHER2_CUBBYFLOW_FBS_H_
#define FLATBUFFERS_GENERATED_POINTSIMPLELISTSEARCHER2_CUBBYFLOW_FBS_H_

#include "flatbuffers/flatbuffers.h"

#include "BasicTypes_generated.h"

namespace CubbyFlow {
namespace fbs {

struct PointSimpleListSearcher2;

struct PointSimpleListSearcher2 FLATBUFFERS_FINAL_CLASS : private flatbuffers::Table {
  enum {
    VT_POINTS = 4
  };
  const flatbuffers::Vector<const CubbyFlow::fbs::Vector2D *> *points() const {
    return GetPointer<const flatbuffers::Vector<const CubbyFlow::fbs::Vector2D *> *>(VT_POINTS);
  }
  bool Verify(flatbuffers::Verifier &verifier) const {
    return VerifyTableStart(verifier) &&
           VerifyOffset(verifier, VT_POINTS) &&
           verifier.Verify(points()) &&
           verifier.EndTable();
  }
};

struct PointSimpleListSearcher2Builder {
  flatbuffers::FlatBufferBuilder &fbb_;
  flatbuffers::uoffset_t start_;
  void add_points(flatbuffers::Offset<flatbuffers::Vector<const CubbyFlow::fbs::Vector2D *>> points) {
    fbb_.AddOffset(PointSimpleListSearcher2::VT_POINTS, points);
  }
  PointSimpleListSearcher2Builder(flatbuffers::FlatBufferBuilder &_fbb)
        : fbb_(_fbb) {
    start_ = fbb_.StartTable();
  }
  PointSimpleListSearcher2Builder &operator=(const PointSimpleListSearcher2Builder &);
  flatbuffers::Offset<PointSimpleListSearcher2> Finish() {
    const auto end = fbb_.EndTable(start_, 1);
    auto o = flatbuffers::Offset<PointSimpleListSearcher2>(end);
    return o;
  }
};

inline flatbuffers::Offset<PointSimpleListSearcher2> CreatePointSimpleListSearcher2(
    flatbuffers::FlatBufferBuilder &_fbb,
    flatbuffers::Offset<flatbuffers::Vector<const CubbyFlow::fbs::Vector2D *>> points = 0) {
  PointSimpleListSearcher2Builder builder_(_fbb);
  builder_.add_points(points);
  return builder_.Finish();
}

inline flatbuffers::Offset<PointSimpleListSearcher2> CreatePointSimpleListSearcher2Direct(
    flatbuffers::FlatBufferBuilder &_fbb,
    const std::vector<const CubbyFlow::fbs::Vector2D *> *points = nullptr) {
  return CubbyFlow::fbs::CreatePointSimpleListSearcher2(
      _fbb,
      points ? _fbb.CreateVector<const CubbyFlow::fbs::Vector2D *>(*points) : 0);
}

inline const CubbyFlow::fbs::PointSimpleListSearcher2 *GetPointSimpleListSearcher2(const void *buf) {
  return flatbuffers::GetRoot<CubbyFlow::fbs::PointSimpleListSearcher2>(buf);
}

inline bool VerifyPointSimpleListSearcher2Buffer(
    flatbuffers::Verifier &verifier) {
  return verifier.VerifyBuffer<CubbyFlow::fbs::PointSimpleListSearcher2>(nullptr);
}

inline void FinishPointSimpleListSearcher2Buffer(
    flatbuffers::FlatBufferBuilder &fbb,
    flatbuffers::Offset<CubbyFlow::fbs::PointSimpleListSearcher2> root) {
  fbb.Finish(root);
}

}  // namespace fbs
}  // namespace CubbyFlow

#endif  // FLATBUFFERS_GENERATED_POINTSIMPLELISTSEARCHER2_CUBBYFLOW_FBS_H_
