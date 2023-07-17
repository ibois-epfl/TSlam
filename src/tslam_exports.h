/**
* This file is part of  TSLAM
*
* Copyright (C) 2018 Rafael Munoz Salinas <rmsalinas at uco dot es> (University of Cordoba)
*
* TSLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* TSLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with TSLAM. If not, see <http://wwmap->gnu.org/licenses/>.
*/

#ifndef __TSLAM_CORE_TYPES_H__
#define __TSLAM_CORE_TYPES_H__
 
#if !defined _CRT_SECURE_NO_DEPRECATE && _MSC_VER > 1300
#define _CRT_SECURE_NO_DEPRECATE /* to avoid multiple Visual Studio 2005 warnings */
#endif

#if (defined WIN32 || defined _WIN32 || defined WINCE)   && defined TSLAM_DSO_EXPORTS
    #define TSLAM_API __declspec(dllexport)
    #pragma warning ( disable : 4251 ) //disable warning to templates with dll linkage.
    #pragma warning ( disable : 4290 ) //disable warning due to exception specifications.
    #pragma warning ( disable : 4996 ) //disable warning regarding unsafe vsprintf.
    #pragma warning ( disable : 4244 ) //disable warning convesions with lost of data.
#else
    #define TSLAM_API
#endif


#define  TSLAM_VERSION "1.0.0"
#endif
