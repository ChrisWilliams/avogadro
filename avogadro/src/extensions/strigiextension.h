#ifndef STRIGIEXTENSION_H
#define STRIGIEXTENSION_H
/**********************************************************************
  Copyright (C) 2007 by Carsten Niehaus

  This file is part of the Avogadro molecular editor project.
  For more information, see <http://avogadro.sourceforge.net/>

  Some code is based on Open Babel
  For more information, see <http://openbabel.sourceforge.net/>

  This program is free software; you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation version 2 of the License.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
 ***********************************************************************/


#include <openbabel/mol.h>
#include <openbabel/generic.h>

#include <avogadro/extension.h>
#include "ui_strigiwidget.h"

#include <QObject>
#include <QList>
#include <QString>
#include <QUndoCommand>

namespace Avogadro {

  class StrigiExtension : public QObject, public Extension
    {
        Q_OBJECT

        public:
            //! Constructor
            StrigiExtension(QObject *parent=0);
            //! Deconstructor
            virtual ~StrigiExtension();

            //! \name Description methods
            //@{
            //! Plugin Name (ie Draw)
            virtual QString name() const { return QObject::tr("Strigi"); }
            //! Plugin Description (ie. Draws atoms and bonds)
            virtual QString description() const { return QObject::tr("Locating Molecule Files with Strigi Plugin"); };
            //! Perform Action
            virtual QList<QAction *> actions() const;
            virtual QUndoCommand* performAction(QAction *action, Molecule *molecule,
                    GLWidget *widget, QTextEdit *messages=NULL);
            //@}

            //  public slots:

        private:
            GLWidget *m_Widget;
            QList<QAction *> m_actions;
            Ui::StrigiWidget *ui;
    };

  class StrigiExtensionFactory : public QObject, public ExtensionFactory
    {
        Q_OBJECT;
        Q_INTERFACES(Avogadro::ExtensionFactory);

        public:
        Extension *createInstance(QObject *parent = 0) { return new StrigiExtension(parent); }
    };

} // end namespace Avogadro

#endif // STRIGIEXTENSION_H
