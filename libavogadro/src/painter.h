/**********************************************************************
  Painter - drawing spheres, cylinders and text in a GLWidget

  Copyright (C) 2007 Benoit Jacob

  This file is part of the Avogadro molecular editor project.
  For more information, see <http://avogadro.sourceforge.net/>

  Avogadro is free software; you can redistribute it and/or modify 
  it under the terms of the GNU General Public License as published by 
  the Free Software Foundation; either version 2 of the License, or 
  (at your option) any later version.

  Avogadro is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
  02110-1301, USA.
 **********************************************************************/

#ifndef __PAINTER_H
#define __PAINTER_H

#include <avogadro/global.h>
#include <eigen/vector.h>
#include <QString>
#include <QPoint>

namespace Avogadro {


  /** @class Painter
    *
    * A simple Painter class for Avogadro. Currently it can draw spheres, cylinders,
    * multiple cylinders (as in multiple bonds) and text.
    *
    * Each GLWidget has a Painter which you can get by calling GLWidget::painter().
    *
    * The motivation for having a centralized Painter class is that it allows for global
    * quality control. Just call setQuality() to adjust the quality setting.
    *
    * Another nice thing is that it handles level-of-detail computation, so that
    * you can just call drawSphere(center,radius) and the Painter automatically
    * determines the correct
    * detail level based on the radius of the sphere, on the distance to the camera, and on
    * the global quality setting.
    *
    * The multiple cylinders are drawn in such a way that the individual cylinders avoid
    * hiding each other, at least in the default viewpoint.
    *
    * The text is drawn with a black outlining so that it remains readable regardless of the
    * background color.
    */
  class A_EXPORT GLWidget;
  class PainterPrivate;
  class A_EXPORT Painter
  {
    public:
      Painter(int quality=-1);
      ~Painter();

      /** sets the GLWidget in which we are painting.
        * Please note that in the current implementation, we are expecting
        * that the font() of the GLWidgets are always the same one (typically,
        * the default font set by the user's desktop environment). Multiple-font
        * text rendering is currently unsupported.
        */
//       void setGLWidget( GLWidget * widget );
      
      /** sets the global quality setting. This influences the detail level of the
        * geometric objects (spheres and cylinders). Values range from 0 to
        * PAINTER_GLOBAL_QUALITY_SETTINGS-1.
        */
      void setQuality( int quality );
      
      /** @returns the current global quality setting.
        */
      int quality() const;
      
      /** You have to call this once, typically in the GLWidget::initializeGL() function.
        * It calls setGLWidget() and setQuality(), and marks this Painter
        * as initialized and ready for painting.
        */
//       void initialize( GLWidget * widget, int quality=-1 );

      /** Draws a sphere with prescribed detail level. The effective detail level
        * is still influenced by the global quality setting. The \a detailLevel argument
        * only makes the detail level independent of the apparent radius of the sphere.
        * @param center The position of the center of the sphere
        * @param radius The radius of the sphere
        * @param detailLevel The detail level, ranging between 0 and PAINTER_DETAIL_LEVELS-1.
        */
      void drawSphere( const Eigen::Vector3d & center, double radius, int detailLevel ) const;
      
      /** Draws a sphere, leaving the Painter choose the appropriate detail level based on the
        * apparent radius (ratio of radius over distance) and the global quality setting.
        * @param center The position of the center of the sphere
        * @param radius The radius of the sphere
        */
      void drawSphere( const Eigen::Vector3d & center, double radius ) const;

      /** Draws a cylinder with prescribed detail level. The effective detail level
        * is still influenced by the global quality setting. The \a detailLevel argument
        * only makes the detail level independent of the apparent radius of the cylinder.
        * @param end1 The position of the first end of the cylinder
        * @param end2 The position of the second end of the cylinder
        * @param radius The radius, i.e. half-width of the cylinder
        * @param detailLevel The detail level, ranging between 0 and PAINTER_DETAIL_LEVELS-1.
        */
      void drawCylinder( const Eigen::Vector3d &end1, const Eigen::Vector3d &end2,
          double radius, int detailLevel ) const;

      /** Draws a cylinder, leaving the Painter choose the appropriate detail level based on the
        * apparent radius (ratio of radius over distance) and the global quality setting.
        * @param end1 The position of the first end of the cylinder
        * @param end2 The position of the second end of the cylinder
        * @param radius The radius, i.e. half-width of the cylinder
        */
      void drawCylinder( const Eigen::Vector3d &end1, const Eigen::Vector3d &end2,
          double radius) const;

      /** Draws a multiple cylinder (see below) with prescribed detail level.
        * The effective detail level
        * is still influenced by the global quality setting. The \a detailLevel argument
        * only makes the detail level independent of the apparent radius of the cylinders.
        *
        * What is a "multiple cylinder" ? Think multiple bond between two atoms.
        * This function is there to allow drawing multiple bonds in a single call.
        *
        * This function takes care of rendering multiple bonds in such a way that the individual
        * bonds avoid hiding each other, at least in the defaut viewpoint on a molecule.
        * To achieves that, it asks the GLWidget for the the normal vector of the
        * molecule's best-fitting plane.
        *
        * @param end1 The position of the first end of the bond
        * @param end2 The position of the second end of the bond
        * @param radius The radius, i.e. half-width of each cylinder
        * @param order The multiplicity order of the bond, e.g. 2 for a double bond.
        *              When this parameter equals 1, this function is equivalent to
        *              drawCylinder().
        * @param shift How far away from the central axis the cylinders are shifted.
        *              In other words, this influences the total width of multiple bonds.
        * @param detailLevel The detail level, ranging between 0 and PAINTER_DETAIL_LEVELS-1.
        */
      void drawMultiCylinder( const Eigen::Vector3d &end1, const Eigen::Vector3d &end2,
          double radius, int order, double shift, int detailLevel ) const;

      /** Draws a multiple cylinder (see below), leaving the Painter choose the appropriate
        * detail level based on the apparent radius (ratio of radius over distance) and the
        * global quality setting.
        *
        * What is a "multiple cylinder" ? Think multiple bond between two atoms.
        * This function is there to allow drawing multiple bonds in a single call.
        *
        * This function takes care of rendering multiple bonds in such a way that the individual
        * bonds avoid hiding each other, at least in the defaut viewpoint on a molecule.
        * To achieves that, it asks the GLWidget for the the normal vector of the
        * molecule's best-fitting plane.
        *
        * @param end1 The position of the first end of the bond
        * @param end2 The position of the second end of the bond
        * @param radius The radius, i.e. half-width of each cylinder
        * @param order The multiplicity order of the bond, e.g. 2 for a double bond.
        *              When this parameter equals 1, this function is equivalent to
        *              drawCylinder().
        * @param shift How far away from the central axis the cylinders are shifted.
        *              In other words, this influences the total width of multiple bonds.
        */
      void drawMultiCylinder( const Eigen::Vector3d &end1, const Eigen::Vector3d &end2,
          double radius, int order, double shift ) const;

      /** Draws a shaded sector of a circle.  The sector is defined by three vectors,
        * the center of the circle, and two vectors that define the lines going out
        * from the centre of the circle to the circumference of the circle.  The
        * actual points on the circumference are found using these two vectors and
        * the radius of the circle.
        *
        * @param origin The center of the circle this sector is a portion of.
        * @param direction1 A vector defining the line the first point will lie on.
        * @param direction2 A vector defining the line the second point will lie on.
        * @param radius The radius of the circle this sector is a portion of.
        */
      void drawShadedSector(Eigen::Vector3d origin, Eigen::Vector3d direction1,
                            Eigen::Vector3d direction2, double radius);

      /** Draws an arc.  The arc is defined by three vectors, the center of the circle,
        * and two vectors that define the lines going out from the center of the
        * circle to the circumference of the circle.  The actual points on the
        * circumference are found using these two vectors and the radius of the circle.
        *
        * @param origin The center of the circle whose circumference this arc is a portion of.
        * @param direction1 A vector defining the line the start of the arc will lie on.
        * @param direction2 A vector defining the line the end of the arc will lie on.
        * @param radius The radius of the circle whose circumference this arc is a portion of.
        * @param lineWidth The thickness of the line the arc will be drawn with.
        */
      void drawArc(Eigen::Vector3d origin, Eigen::Vector3d direction1, Eigen::Vector3d direction2,
                   double radius, double lineWidth);

      /** Draws a solid two dimensional quadrilateral in three dimensional space.
        *
        * @param point1 The first of the four corners of the quadrilateral.
        * @param point2 The second of the four corners of the quadrilateral.
        * @param point3 The third of the four corners of the quadrilateral.
        * @param point4 The last of the four corners of the quadrilateral.
        */
      void drawShadedQuadrilateral(Eigen::Vector3d point1, Eigen::Vector3d point2,
                             Eigen::Vector3d point3, Eigen::Vector3d point4);

      /** Draws the outline of a two dimensional quadrilateral in three dimensional space.
        *
        * @param point1 The first of the four corners of the quadrilateral.
        * @param point2 The second of the four corners of the quadrilateral.
        * @param point3 The third of the four corners of the quadrilateral.
        * @param point4 The last of the four corners of the quadrilateral.
        * @param lineWidth The thickness of the line the quadrilateral will be drawn with.
        */
      void drawQuadrilateral(Eigen::Vector3d point1, Eigen::Vector3d point2,
                             Eigen::Vector3d point3, Eigen::Vector3d point4,
                             double lineWidth);

      /** Draws text at a given window position, on top of the scene.
        * @note Calls to drawText methods must be enclosed between begin() and end().
        * @note Text is rendered as a transparent object, and should therefore be rendered after
        *       the opaque objects.
        * @param x,y the window coordinates of the top-left corner of the text to render.
        *        (0,0) is the top-left corner of the window.
        * @param string The string to render. All character encodings are allowed, but
        *               superposed characters are not supported yet. For accented letters,
        *               use a character giving the whole accented letter, not a separate
        *               character for the accent.
        * @sa begin(), drawText( const Eigen::Vector3d &, const QString &) const,
        *     drawText( const QPoint &, const QString & ) const
        */
      int drawText( int x, int y, const QString &string ) const;
      
      /** Draws text at a given window position, on top of the scene.
        * @note Calls to drawText methods must be enclosed between begin() and endText().
        * @note Text is rendered as a transparent object, and should therefore be rendered after
        *       the opaque objects.
        * @param pos the window coordinates of the top-left corner of the text to render.
        *        (0,0) is the top-left corner of the window.
        * @param string The string to render. All character encodings are allowed, but
        *               superposed characters are not supported yet. For accented letters,
        *               use a character giving the whole accented letter, not a separate
        *               character for the accent.
        * @sa begin(), drawText( const Eigen::Vector3d &, const QString &) const,
        *     drawText( int, int, const QString & ) const
        */
      int drawText( const QPoint& pos, const QString &string ) const;
      
      /** Draws text at a given scene position, inside the scene.
        * @note Calls to drawText methods must be enclosed between begin() and endText().
        * @note Text is rendered as a transparent object, and should therefore be rendered after
        *       the opaque objects.
        * @param pos the scene coordinates of the top-left corner of the text to render.
        * @param string The string to render. All character encodings are allowed, but
        *               superposed characters are not supported yet. For accented letters,
        *               use a character giving the whole accented letter, not a separate
        *               character for the accent.
        * @sa begin(), drawText( const QPoint&, const QString &) const,
        *     drawText( int, int, const QString & ) const
        */
      int drawText( const Eigen::Vector3d & pos, const QString &string ) const;
      
      /** Enter text-drawing mode. You must call this before calling any of the drawText functions.
        * You must then call endText() to leave text-drawing mode. Be careful when doing OpenGL
        * state changes while in text drawing mode. Of course, changing the drawing color is safe,
        * as well as modifying the modelview matrix.
        * @sa endText()
        */
      void begin(GLWidget *widget);
      /** Leave text-drawing mode.
        * @sa begin()
        */
      void end();


      static int defaultQuality();
      static int maxQuality();

    private:
      PainterPrivate * const d;
  };

} // end namespace Avogadro

#endif // _PAINTER_H
