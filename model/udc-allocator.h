/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2007 INRIA
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Mathieu Lacage <mathieu.lacage@sophia.inria.fr>
 *//* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
#ifndef UDC_ALLOCATOR_H
#define UDC_ALLOCATOR_H

#include "ns3/node-container.h"
#include "ns3/position-allocator.h"
#include "ns3/random-variable-stream.h"
#include "ns3/vector.h"

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>

namespace ns3 {

//
// I have copied the structure of this object from
// the ListPositionAllocator and modified as needed.
// Once this is working, I would like to move the
// implementation out of ns-3's code and into a
// module that users could download to use in
// their ns-3 installation without modifying ns-3
// directly.
//

/**
 * \ingroup mobility
 * \brief Allocate positions according to a unit disk cover algorithm.
 *
 * The user will provide a position allocator and the disk size.
 * The first call to ListPositionAllocator::GetNext will return the
 * first disk placed by the algorithm, the second call, the second disk,
 * and so on.
 */
class UDCPositionAllocator : public PositionAllocator
{
public:
   enum Algorithm {
	   FAST_COVER = 0, SWEEP, STRIPS
   };
   typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
   typedef Kernel::Point_3   Point_3;
   typedef Kernel::Point_2   Point_2;
   typedef Kernel::Segment_2 Segment_2;

  /**
   * Register this type with the TypeId system.
   * \return the object TypeId
   */
  static TypeId GetTypeId (void);

  void SetAlgorithm (int method);

  void SetSites (NodeContainer c);

  /**
   * \brief Compute a unit disk cover approximation to cover the points
   * \param allocator the points that must be covered
   * \param radius the radius of the unit disk or coverage area
   */
  void CoverSites (double radius);

  void Print ();

  /**
   * Return the number of positions stored.  Note that this will not change
   * based on calling GetNext(), as the number of positions is not altered
   * by calling GetNext ().
   *
   * \return the number of positions stored
   */
  uint32_t GetSize (void) const;
  uint32_t GetSitesN (void) const;
  virtual Vector GetNext (void) const;
  virtual int64_t AssignStreams (int64_t stream);
private:

  /**
   * \brief Perform the Ghosh et al algorithm on the sites in the given node container
   */
  void FastCover (double radius);

  /**
   * \brief Perform the Biniaz et al algorithm on the sites in the given node container
   */
  void BLMS (double radius);

  /**
   * \brief Perform the Liu-Li algorithm on the sites in the given node container
   */
  void LL (double radius);

  /**
   * \brief Add a position to the list of positions
   * \param v the position to append at the end of the list of positions to return from GetNext.
   */
  void Add (Vector v);
  double SquaredDistance (const Vector& l, const Vector& r);

  Algorithm m_method = Algorithm(0); // set default to the first value given in the enum
  size_t m_maxCoverageSites = 50000;
  double m_defaultHeight = 1.2;
  double m_radius; //!< the radius of the unit disk (coverage area)
  std::vector<Vector> m_bounds; //!< the bounds of the given collection of sites
  std::vector<Vector> m_sites; //!< sites to cover
  std::vector<Vector> m_positions;  //!< vector of positions
  mutable std::vector<Vector>::const_iterator m_current; //!< vector iterator
};


} // namespace ns3

#endif /* UDC_ALLOCATOR_H */
