// Copyright (c) 2007  GeometryFactory (France).  All rights reserved.
//
// This file is part of CGAL (www.cgal.org)
//
// $URL: https://github.com/CGAL/cgal/blob/v5.5.1/Triangulation_2/include/CGAL/boost/graph/graph_traits_Triangulation_2.h $
// $Id: graph_traits_Triangulation_2.h 0779373 2020-03-26T13:31:46+01:00 Sébastien Loriot
// SPDX-License-Identifier: LGPL-3.0-or-later OR LicenseRef-Commercial
//
//
// Author(s)     : Andreas Fabri, Fernando Cacciola

#ifndef CGAL_GRAPH_TRAITS_TRIANGULATION_2_H
#define CGAL_GRAPH_TRAITS_TRIANGULATION_2_H

#include <CGAL/Triangulation_2.h>
#include <CGAL/boost/graph/properties_Triangulation_2.h>

// The functions and classes in this file allows the user to
// treat a CGAL Triangulation_2 object as a boost graph "as is". No
// wrapper is needed for the Triangulation_2 object.

#define CGAL_2D_TRIANGULATION_TEMPLATE_PARAMETERS typename GT, typename TDS
#define CGAL_2D_TRIANGULATION CGAL::Triangulation_2<GT, TDS>
#define CGAL_2D_TRIANGULATION_TEMPLATES GT, TDS

#include <CGAL/boost/graph/internal/graph_traits_2D_triangulation.h>

#endif // CGAL_GRAPH_TRAITS_TRIANGULATION_2_H
