/**********************************************************************
  Copyright (C) 2007 by CArsten Niehaus

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

#include "strigiextension.h"
#include <avogadro/primitive.h>
#include <avogadro/color.h>

#include "ui_strigiwidget.h"

#include <QtGui>
#include <QDialog>

using namespace std;
using namespace OpenBabel;

namespace Avogadro {
  StrigiExtension::StrigiExtension(QObject *parent) : QObject(parent), m_Widget(NULL)
  {
    QAction *action = new QAction(this);
    action->setText("Search Molecules...");
    m_actions.append(action);

    QDialog *t = new QDialog();

    ui = new Ui::StrigiWidget();
    ui->setupUi(t);
    t->show();
 
//X     connect(m_Dialog, SIGNAL(StrigiDisplayChanged(int, int, int)),
//X             this, SLOT(StrigiDisplayChanged(int, int, int)));
//X     connect(m_Dialog, SIGNAL(StrigiParametersChanged(double, double, double, double, double, double)),
//X             this, SLOT(StrigiParametersChanged(double, double, double, double, double, double)));
  }

  StrigiExtension::~StrigiExtension()
  {
  }

  QList<QAction *> StrigiExtension::actions() const
  {
    return m_actions;
  }

  QUndoCommand* StrigiExtension::performAction(QAction *,
                                                 Molecule *molecule,
                                                 GLWidget *widget,
                                                 QTextEdit *)
  {
      qDebug() << "perform action";


    return NULL;
  }
} // end namespace Avogadro

#include "strigiextension.moc"
Q_EXPORT_PLUGIN2(strigiextension, Avogadro::StrigiExtensionFactory)
