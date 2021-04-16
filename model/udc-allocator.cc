/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2021
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
 * Author: Matthew Graham <mattgrahamatc@gmail.com>
 */
#include "udc-allocator.h"

#include "ns3/double.h"
#include "ns3/string.h"
#include "ns3/pointer.h"
#include "ns3/uinteger.h"
#include "ns3/enum.h"
#include "ns3/log.h"
#include "ns3/vector.h"
#include "ns3/mobility-model.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <set>
#include <unordered_set>
#include <utility>

#include <CGAL/squared_distance_3.h>

namespace ns3 {

NS_OBJECT_ENSURE_REGISTERED (UDCPositionAllocator);

struct pair_hash {
    inline size_t operator()(const std::pair<int,int> &v) const {
        return v.first*31+v.second;
    }
};

struct sortByX {
	template<typename Point>
	bool operator()(const Point &p1, const Point &p2) {
		return p1.x() < p2.x();
	}
};
inline bool
isEven(int n) {
    return (n % 2 == 0);
}

inline bool
isPresent(const std::unordered_set<std::pair<int,int>, pair_hash> &table, const int vertical, const int horizontal) {
    return table.find(std::make_pair(vertical,horizontal)) != table.end();
}

bool
operator== (const Vector& l, const Vector& r) {
	return l.x == r.x && l.y == r.y && l.z == r.z;
}

inline UDCPositionAllocator::Point_3
CreatePointFromVector (const Vector& v) {
	return UDCPositionAllocator::Point_3(v.x,v.y,v.z);
}

inline double
UDCPositionAllocator::SquaredDistance (const Vector& l, const Vector& r) {
	return CGAL::squared_distance(
			CreatePointFromVector (l),
			CreatePointFromVector (r)
	);
}

TypeId
UDCPositionAllocator::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::UDCPositionAllocator")
    .SetParent<PositionAllocator> ()
    .SetGroupName ("Mobility")
    .AddConstructor<UDCPositionAllocator> ()
  ;
  return tid;
}

void
UDCPositionAllocator::SetAlgorithm (int method)
{
	m_method = Algorithm(method);
}

void
UDCPositionAllocator::SetSites (NodeContainer c)
{
	NodeContainer::Iterator i = c.Begin ();
	Vector firstPosition = (*i)->GetObject<MobilityModel> ()->GetPosition ();

	double MinX = firstPosition.x,
		   MaxX = firstPosition.x,
		   MinY = firstPosition.y,
		   MaxY = firstPosition.y,
		   MinZ = firstPosition.z,
		   MaxZ = firstPosition.z;


	m_sites.reserve (c.GetN());
	using std::min;
	using std::max;
	using std::cout;


	for ( ; i != c.End (); ++i)
	{
		Ptr<MobilityModel> edMobility = (*i)->GetObject<MobilityModel> ();
		Vector position = edMobility->GetPosition ();
		m_sites.push_back(position);

		MinX = min (MinX, position.x);
		MaxX = max (MaxX, position.x);
		MinY = min (MinY, position.y);
		MaxY = max (MaxY, position.y);
		MinZ = min (MinZ, position.z);
		MaxZ = max (MaxZ, position.z);
		//std::cout<<position<<"\n";
	}
	m_bounds = {
			{ MinX, MinY, MinZ },
			{ MaxX, MaxY, MaxZ }
	};
}

void
UDCPositionAllocator::CoverSites ( double radius )
{
  m_radius = radius;
  switch(m_method) {
  case Algorithm::SWEEP:
	  BLMS (radius);
	  break;
  case Algorithm::STRIPS:
	  LL (radius);
	  break;
  case Algorithm::FAST_COVER:
  default:
	  FastCover (radius);
  }
}
void
UDCPositionAllocator::FastCover (double radius) {
	/*
	 * Code and algorithm from
	 *
	 * Ghosh, A., Hicks, B., Shevchenko, R. (2017):
	 * Unit Disk Cover for Massive Point Sets.
	 * In: Kotsireas I., Pardalos P., Parsopoulos
	 * K., Souravlias D., Tsokas A. (eds) Analysis
	 * of Experimental Algorithms. SEA 2019.
	 * Lecture Notes in Computer Science, vol
	 * 11544. Springer, Cham.
	 * https://doi.org/10.1007/978-3-030-34029-2_10.
	 */
	using std::cout;

    std::unordered_set< std::pair<int,int>, pair_hash> hashTableForLatticeDiskCenters;
    const double sqrt2 = std::sqrt(2);
    const double gridWidth = sqrt2 * radius;
    const double additiveFactor = gridWidth/2;
  //  const double onePointFiveOverSqrt2 =  1.5/sqrt2;
//    const double oneOverTwoSqrt2 = 1/(2*sqrt2);
    const double gridWidthTimesOnePointFive = gridWidth * 1.5;
    const double gridWidthTimesZeroPointFive = gridWidth * 0.5;

    double verticalTimesGridWidth, horizontalTimesGridWidth;
    int vertical, horizontal;

    for( Vector p : m_sites ) {
    	//std::cout<<p<<"\n";
        vertical = floor(p.x/gridWidth);
        horizontal = floor(p.y/gridWidth);
        verticalTimesGridWidth  = vertical * gridWidth;
        horizontalTimesGridWidth  = horizontal * gridWidth;

        if( isPresent(hashTableForLatticeDiskCenters,vertical,horizontal))
           continue;

        if( p.x >= verticalTimesGridWidth + gridWidthTimesOnePointFive
         && isPresent( hashTableForLatticeDiskCenters, vertical+1, horizontal )
         && !(SquaredDistance( p, Vector(gridWidth*(vertical+1)+additiveFactor, horizontalTimesGridWidth+additiveFactor, 0) )>1) )
            continue;

        if( p.x <= verticalTimesGridWidth - gridWidthTimesZeroPointFive
         && isPresent( hashTableForLatticeDiskCenters, vertical-1, horizontal )
         && !(SquaredDistance( p, Vector(gridWidth*(vertical-1)+additiveFactor, horizontalTimesGridWidth+additiveFactor, 0) )>1) )
            continue;

        if( p.y <= horizontalTimesGridWidth - gridWidthTimesZeroPointFive
         && isPresent( hashTableForLatticeDiskCenters, vertical, horizontal-1 )
         && !(SquaredDistance( p, Vector(verticalTimesGridWidth+additiveFactor, gridWidth*(horizontal-1)+additiveFactor, 0) )>1) )
            continue;

        if( p.y >= horizontalTimesGridWidth + gridWidthTimesOnePointFive
         && isPresent( hashTableForLatticeDiskCenters, vertical, horizontal+1 )
         && !(SquaredDistance( p, Vector(verticalTimesGridWidth+additiveFactor, gridWidth*(horizontal+1)+additiveFactor, 0) )>1) )
            continue;

        hashTableForLatticeDiskCenters.insert(std::pair<int,int>(vertical,horizontal));
        Vector diskCenter (verticalTimesGridWidth+additiveFactor,
        		           horizontalTimesGridWidth+additiveFactor,
						   m_defaultHeight);
        Add (diskCenter);
        //cout<<diskCenter<<"\n";
    }
}
void
UDCPositionAllocator::BLMS (double radius) {
	/*
	 * Algorithm from
	 *
	 * Biniaz, A., Liu, P., Maheshwari, A., Smid, M.:
	 * Approximation algorithms for the unit disk cover
	 * problem in 2D and 3D. Comput. Geom. 60, 8–18 (2017).
	 *
	 * Code adapted from
	 *
	 * Ghosh, A., Hicks, B., Shevchenko, R. (2017):
	 * Unit Disk Cover for Massive Point Sets.
	 * In: Kotsireas I., Pardalos P., Parsopoulos
	 * K., Souravlias D., Tsokas A. (eds) Analysis
	 * of Experimental Algorithms. SEA 2019.
	 * Lecture Notes in Computer Science, vol
	 * 11544. Springer, Cham.
	 * https://doi.org/10.1007/978-3-030-34029-2_10.
	 */

	const double radius_squared = pow( radius, 2 );

	typedef std::vector<Point_2> PointContainer;
	typedef PointContainer::iterator PointIterator;

	// Sort all points on x-coordinate
	PointContainer P;
	P.reserve(m_sites.size());
	for( auto v : m_sites ) {
		P.emplace_back( v.x, v.y );
	}
	std::sort( P.begin(), P.end(), [] ( const Point_2 &lhs, const Point_2 &rhs ) {
		return lhs.x() < rhs.x();
	});

	// Create the BST
	auto YItSorter = []( const PointIterator &lhs, const PointIterator &rhs ) {
		return lhs->y() < rhs->y();
	};
    std::set<PointIterator,decltype(YItSorter)> BST(YItSorter); // the binary tree of y-sorted disks

    // Predicate to tell if a point is covered by a disk
	auto isCovered = [&]( const Point_2& p, const Point_2& q ) {
	    return CGAL::squared_distance( p, q ) < radius_squared;
	};

	for( auto sit=P.begin(), dit=P.begin(); sit<P.end(); sit++ ) {
		// Handle deletions
		while( dit->x() + radius < sit->x() ) {
			BST.erase(dit);
			dit++;
		}

		// Handle site
		auto p_plus = BST.lower_bound(sit),
			 pos(p_plus); // one higher than the site (p+)

		bool siteIsCovered = false;

		while( pos != BST.end() && (*pos)->y() - sit->y() < radius && !siteIsCovered ) {
			siteIsCovered = isCovered( *sit, **pos );
			pos++;
		}
		pos = p_plus;

		while( !siteIsCovered && pos != BST.begin() && sit->y() - (*--pos)->y() < radius ) {
			siteIsCovered = isCovered( *sit, **pos );
		}

		if( !siteIsCovered ) {
			// add disk centers to C
	        Add (Vector( sit->x(), sit->y(), m_defaultHeight ));
			BST.insert(sit); // insert the Point into the BST
		}

	}
}
void
UDCPositionAllocator::LL (double radius) {
	/*
	 * Algorithm from
	 *
	 * Liu, P., Lu, D.: A fast 25/6-approximation for the
	 * minimum unit disk cover problem. arXiv preprint
	 * arXiv:1406.3838 (2014).
	 *
	 * Code adapted from
	 *
	 * Ghosh, A., Hicks, B., Shevchenko, R. (2017):
	 * Unit Disk Cover for Massive Point Sets.
	 * In: Kotsireas I., Pardalos P., Parsopoulos
	 * K., Souravlias D., Tsokas A. (eds) Analysis
	 * of Experimental Algorithms. SEA 2019.
	 * Lecture Notes in Computer Science, vol
	 * 11544. Springer, Cham.
	 * https://doi.org/10.1007/978-3-030-34029-2_10.
	 */

	using std::list;
	using std::vector;

	typedef std::vector<Point_2> PointContainer;
	//typedef PointContainer::iterator PointIterator;

	// Sort all points on x-coordinate
	PointContainer P;
	P.reserve(m_sites.size());
	for( auto v : m_sites ) {
		P.emplace_back( v.x, v.y );
	}

	const long double sqrt3TimesRadius = std::sqrt(3)*radius, sqrt3TimesRadiusOver2 = sqrt3TimesRadius/2;
	unsigned answer = P.size()+1;
	sort(P.begin(),P.end(),sortByX());
	list<Point_2> C;


	for(unsigned i = 0; i < 6; i++) {

		unsigned current = 0;
		long double rightOfCurrentStrip = P[0].x()  + ((i*sqrt3TimesRadius)/6);
		list<Point_2> tempC;

		while( current < P.size() ) {

			if(P[current].x() > rightOfCurrentStrip) {
				int jump = (P[current].x() - rightOfCurrentStrip) / sqrt3TimesRadius;
				rightOfCurrentStrip += jump*sqrt3TimesRadius;

				if(jump > 0)
					continue;
			}

			unsigned indexOfTheFirstPointInTheCurrentStrip = current;

			while(P[current].x() < rightOfCurrentStrip && current < P.size())
				current++;


			vector<Segment_2> segments;
			segments.reserve(current-indexOfTheFirstPointInTheCurrentStrip+1);
			long double xOfRestrictionline = rightOfCurrentStrip - sqrt3TimesRadiusOver2;

			for(unsigned j = indexOfTheFirstPointInTheCurrentStrip; j < current; j++) {
				long double distanceFromRestrictionLine = P[j].x()-xOfRestrictionline;
				long double y = std::sqrt(pow(radius,2)-( distanceFromRestrictionLine*distanceFromRestrictionLine));
				segments.emplace_back( Point_2(xOfRestrictionline,P[j].y()+y), Point_2(xOfRestrictionline,P[j].y()-y) );
			}

			rightOfCurrentStrip += sqrt3TimesRadius;

			if(segments.size() == 0)
				continue;

			sort(segments.begin(),segments.end(), [](const Segment_2& si, const Segment_2& sj) { return (si.target().y() > sj.target().y());});

			long double lowestY = segments[0].target().y();

			for( unsigned k = 1; k < segments.size(); k++) {
				if( segments[k].source().y() < lowestY ) {
					tempC.emplace_back(xOfRestrictionline,lowestY);
					lowestY = segments[k].target().y();
				}
			}
			tempC.emplace_back(xOfRestrictionline,lowestY);
		}

	   //cout << tempC.size() << endl;

		if( tempC.size() < answer) {
			answer = tempC.size();
			std::swap(C,tempC);
		}
	}
	using std::cout;
	cout<<"\n";
	for( Point_2 p : C ) {
        Add (Vector( p.x(), p.y(), m_defaultHeight ));
        cout<<p<<"\n";
	}
}

void
UDCPositionAllocator::Print()
{
	using std::string;
	using std::max;
	using std::cout;
	using std::endl;

	string FileName = "temp";
	string TexFileName = FileName + ".tex";
    std::FILE *fp = fopen (TexFileName.c_str() ,"w");

    fprintf (fp,"\\documentclass{standalone} \n\\usepackage{tikz} \n \n\n\\begin{document}\n");
    fprintf (fp,"\n\n\n\\begin{tikzpicture}\n\n");

    // Print origin
    //fprintf(fp,"\\draw [fill=white,stroke=green] (0,0) circle [radius=%f];\n",radiusOfPoints);

    const double ResultantDimension = 20;

    double ResizeFactor = ResultantDimension
    			 	 	     / max (m_bounds[1].x-m_bounds[0].x,
    						        max (m_bounds[1].y-m_bounds[0].y,
    								     m_bounds[1].z-m_bounds[0].z));

    const double RadiusOfPoints = 0.005*ResultantDimension;

    // Print coverage disks
    for(Vector p : m_positions){
        fprintf (fp, "\\draw [color=red!60, fill=red, fill opacity=0.05](%f,%f) circle [radius=%f];\n",
        		 p.x*ResizeFactor, p.y*ResizeFactor, m_radius*ResizeFactor);
    }
    // Print sites
    for(Vector p : m_sites)
        fprintf (fp, "\\draw [color=blue, fill=blue!63] (%f,%f) circle [radius=%f];\n",
        		p.x*ResizeFactor, p.y*ResizeFactor, RadiusOfPoints);

    // Draw scale
    double scaleStartX = (m_bounds[1].x - 2*m_radius)*ResizeFactor,
    	   scaleLabelX = scaleStartX + m_radius*ResizeFactor,
    	   scaleEndX   = scaleLabelX + m_radius*ResizeFactor,
		   scaleY	   = (m_bounds[1].y)*ResizeFactor + 0.5;
    string scaleLabel = std::to_string(int(2*m_radius)) + "m";
    fprintf (fp, "\\draw [|-|, ultra thick](%f,%f) -- (%f,%f) node[anchor=north]{%s} -- (%f,%f) ;\n",
    		scaleStartX,scaleY,scaleLabelX,scaleY, scaleLabel.c_str(), scaleEndX,scaleY);

    fprintf (fp, "\n\n\\end{tikzpicture}");
    fprintf (fp, "\n\n\\end{document}");
    fclose (fp);

    cout << "\nOutput PDF generation started..." << endl;
    string command = "pdflatex " + TexFileName + " > /dev/null";
    system (command.c_str());
    cout << "PDF generation terminated..." << endl;

    command = "atril " + FileName + ".pdf &";
    system (command.c_str());
}

void
UDCPositionAllocator::Add (Vector v)
{
  m_positions.push_back (v);
  m_current = m_positions.begin ();

  m_bounds[0].x = std::min (m_bounds[0].x, v.x-m_radius);
  m_bounds[0].y = std::min (m_bounds[0].y, v.y-m_radius);
  m_bounds[0].z = std::min (m_bounds[0].z, v.z-m_radius);
  m_bounds[1].x = std::max (m_bounds[1].x, v.x+m_radius);
  m_bounds[1].y = std::max (m_bounds[1].y, v.y+m_radius);
  m_bounds[1].z = std::max (m_bounds[1].z, v.z+m_radius);
}

Vector
UDCPositionAllocator::GetNext (void) const
{
  Vector v = *m_current;
  m_current++;
  if (m_current == m_positions.end ())
    {
      m_current = m_positions.begin ();
    }
  return v;
}
int64_t
UDCPositionAllocator::AssignStreams (int64_t stream)
{
  return 0;
}

uint32_t
UDCPositionAllocator::GetSize (void) const
{
  return m_positions.size ();
}

uint32_t
UDCPositionAllocator::GetSitesN (void) const
{
  return m_sites.size ();
}


} // namespace ns3 
