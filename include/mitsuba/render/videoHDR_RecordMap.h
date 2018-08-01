/* ---------------------------------------------------------------------------
** videoHDR_Record.h
** This file implements the records map to be used by the VideoHDR integrator
**
** Author: Ricardo Marques (based on photonmap.h)
** Date: March, 2014
** -------------------------------------------------------------------------*/

#pragma once
#if !defined(__MITSUBA_RENDER_VIDEOHDR_RECORDMAP_H_)
#define __MITSUBA_RENDER_VIDEOHDR_RECORDMAP_H_

#include <mitsuba/render/videoHDR_Record.h>

MTS_NAMESPACE_BEGIN

class MTS_EXPORT_RENDER videoHDR_RecordMap : public SerializableObject {
public:
	typedef PointKDTree<VideoHDR_Record>        VideoHDR_RecordTree;
	typedef VideoHDR_RecordTree::IndexType      IndexType;
	typedef VideoHDR_RecordTree::SearchResult   SearchResult;

    /* ===================================================================== */
    /*                        Public access methods                          */
    /* ===================================================================== */

	/**
	 * \brief Create an empty HDR records map and reserve memory
	 * for a specified number of HDR records.
	 */
	videoHDR_RecordMap(size_t HDR_RecordsCount = 0);

	/**
	 * \brief Unserialize an HDR record map from a binary data stream
	 */
	videoHDR_RecordMap(Stream *stream, InstanceManager *manager);

	// =============================================================
	//! @{ \name \c stl::vector-like interface
	// =============================================================
	/// Clear the kd-tree array
	inline void clear() { m_kdtree.clear(); }
	/// Resize the kd-tree array
	inline void resize(size_t size) { m_kdtree.resize(size); }
	/// Reserve a certain amount of memory for the kd-tree array
	inline void reserve(size_t size) { m_kdtree.reserve(size); }
	/// Return the size of the kd-tree
	inline size_t size() const { return m_kdtree.size(); }
	/// Return the capacity of the kd-tree
	inline size_t capacity() const { return m_kdtree.capacity(); }
	/// Append a kd-tree photon to the photon array
	inline void push_back(const VideoHDR_Record &hdrRecord) { m_kdtree.push_back(hdrRecord); }
	/// Return one of the photons by index
	inline VideoHDR_Record &operator[](size_t idx) { return m_kdtree[idx]; }
	/// Return one of the photons by index (const version)
	inline const VideoHDR_Record &operator[](size_t idx) const { return m_kdtree[idx]; }
	//! @}
	// =============================================================

	// =============================================================
	//! @{ \name \c VideoHDR map query functions
	// =============================================================

	/// Perform a nearest-neighbor query, see \ref PointKDTree for details
	inline size_t nnSearch(const Point &p, Float &sqrSearchRadius,
		size_t k, SearchResult *results) const {
		return m_kdtree.nnSearch(p, sqrSearchRadius, k, results);
	}

	/// Perform a nearest-neighbor query, see \ref PointKDTree for details
	inline size_t nnSearch(const Point &p,
		size_t k, SearchResult *results) const {
		return m_kdtree.nnSearch(p, k, results);
	}
	//! @}
	// =============================================================


	/**
	 * \brief Try to append a hdrRecord to the hdrRecord map
	 *
	 * \return \c false If the hdrRecord map is full
	 */
	inline bool tryAppend(const VideoHDR_Record &hdrRecord) {
		if (size() < capacity()) {
			push_back(hdrRecord);
			return true;
		} else {
			return false;
		}
	}

	/**
	 * \brief Build a record map over the supplied records.
	 *
	 * This has to be done once after all photons have been stored,
	 * but prior to executing any queries.
	 */
	inline void build(bool recomputeAABB = false) { m_kdtree.build(recomputeAABB); }

	/// Return the depth of the constructed KD-tree
	inline size_t getDepth() const { return m_kdtree.getDepth(); }

	/// Determine if the record map is completely filled
	inline bool isFull() const {
		return capacity() == size();
	}

	/// Serialize a hdr record map to a binary data stream
	void serialize(Stream *stream, InstanceManager *manager) const;

	/// Dump the hdr records to an OBJ file to analyze their spatial distribution
	void dumpOBJ(const std::string &filename);

	/// Return a string representation
	std::string toString() const;

	MTS_DECLARE_CLASS()
protected:
	/// Virtual destructor
	virtual ~videoHDR_RecordMap();
protected:
	VideoHDR_RecordTree m_kdtree;
	//Float m_scale;
};

MTS_NAMESPACE_END

#endif /* __MITSUBA_RENDER_VIDEOHDR_RECORDMAP_H_ */
