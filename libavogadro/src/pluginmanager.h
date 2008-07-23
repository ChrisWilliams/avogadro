/**********************************************************************
  PluginManager - Class to handle dynamic loading/unloading of plugins

  Copyright (C) 2008 Tim Vandermeersch

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

#ifndef PLUGINMANAGER_H
#define PLUGINMANAGER_H

#include <avogadro/global.h>
#include <avogadro/engine.h>
#include <avogadro/tool.h>
#include <avogadro/extension.h>
#include <avogadro/color.h>

#include <QSettings>

namespace Avogadro {

  class PluginItemPrivate;
  class PluginItem 
  {
    public:
      PluginItem();
      PluginItem(int type, const QString &fileName, const QString &filePath);
      ~PluginItem();

      /**
       * The plugin type (engine = 0, tool = 1, extension = 2)
       */
      int type() const;
      /**
       * The plugin name (Draw, Stick, ...)
       */
      QString name() const;
      /**
       * The plugin description (Draw, Stick, ...)
       */
      QString description() const;
      /**
       * The plugin filename (libdrawtool.so, libaligntool.dll, ...)
       */ 
      QString fileName() const;
      /**
       * The absolute file path
       */
      QString absoluteFilePath() const;
      /**
       * Should the plugin be loaded
       */
      bool isEnabled() const;

      /**
       * Set the plugin type (engine = 0, tool = 1, extension = 2)
       */
      void setType( int type );
      /**
       * Set the plugin name (Draw, Stick, ...)
       */
      void setName( const QString &name );
      /**
       * Set the plugin description
       */
      void setDescription( const QString &description );
      /**
       * The plugin filename (libdrawtool.so, libaligntool.dll, ...)
       */ 
      void setFileName( const QString &fileName );
      /**
       * The absolute file path
       */
      void setAbsoluteFilePath( const QString &filePath );
      /**
       * Should the plugin be loaded
       */
      void setEnabled( bool enable );

    private:
      PluginItemPrivate * const d;
  };

  class PluginManagerPrivate;
  class A_EXPORT PluginManager: public QObject
  {
    Q_OBJECT

  public:
    PluginManager(QObject *parent = 0);
    ~PluginManager();

    /**
     * Get all the PluginItems for a given type
     */
    QList<PluginItem *> plugins( int type );
 
    /**
     * Find all plugins by looking through the search paths:
     *    /usr/(local/)lib/avogadro/engines
     *    /usr/(local/)lib/avogadro/tools
     *    /usr/(local/)lib/avogadro/extensions
     *    /usr/(local/)lib/avogadro/colors
     *
     * You can set AVOGADRO_ENGINES, AVOGADRO_TOOLS, AVOGADRO_EXTENSIONS 
     * and AVOGADRO_COLORS to designate a path at runtime.
     *
     * WIN32: look in the applications working dir ( ./engines, ...)
     */ 
    void loadPlugins();
    
    /**
     * Get the loaded engine factories
     */
    const QList<PluginFactory *>& engineFactories() const;
    /**
     * Get the QHash to translate an engine className to 
     * a PluginFactory* pointer.
     */
    const QHash<QString, PluginFactory *>& engineClassFactory() const;
    /**
     * Get the loaded tools
     */
    const QList<Tool *>& tools() const;
    /**
     * Get the loaded extensions
     */
    const QList<Extension *>& extensions() const;
    /**
     * Get the loaded color plugins
     */
    const QList<Color *>& colors() const;

    /**
     * Write the settings of the PluginManager in order to save them to disk.
     */
    void writeSettings(QSettings &settings) const;
  
  public Q_SLOTS:
    void showDialog();
 
  private:
    PluginManagerPrivate * const d;
  };

  A_DECL_EXPORT extern PluginManager pluginManager;

}

#endif
