//######################################################################
//
// GraspIt!
// Copyright (C) 2002-2009  Columbia University in the City of New York.
// All rights reserved.
//
// GraspIt! is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// GraspIt! is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with GraspIt!.  If not, see <http://www.gnu.org/licenses/>.
//
// Author(s):  Hao Dang and Matei T. Ciocarlie
//
// $Id: dbaseDlg.cpp,v 1.31 2010/09/01 23:54:19 cmatei Exp $
//
//######################################################################

/*! \file
  \brief Defines the %DBaseDlg class
 */
#include "dbaseDlg.h"

#include <algorithm>

#include <utility>
#include <QFileDialog>
#include <QDir>
#include <QComboBox>

#include "graspitGUI.h"
#include "ivmgr.h"
#include "robot.h"
#include "world.h"
#include "searchState.h"
#include "grasp.h"
#include "graspitGUI.h"
#include "mainWindow.h"
#include "matvec3D.h"


#include "DBPlanner/sql_database_manager.h"
#ifdef ROS_DATABASE_MANAGER
#include "DBPlanner/ros_database_manager.h"
#endif

#include "graspit_db_model.h"
#include "graspit_db_grasp.h"
#include "dbasePlannerDlg.h"
#include "graspCaptureDlg.h"


#include <include/EGPlanner/egPlanner.h>
#include <include/EGPlanner/simAnnPlanner.h>
#include <include/EGPlanner/searchState.h>
#include <include/EGPlanner/searchEnergy.h>

#include "quality.h"
#include <unistd.h>


//#define GRASPITDBG
#include "debug.h"

/*! Initializes the dialog and also gets the one and only manager from the
    GraspitGUI. If this manager is already set, it also loads the model
    list from the database and initializes it.
*/
void DBaseDlg::init()
{
    mModelList.clear();
    mGraspList.clear();
    mGraspList.clear();
    browserGroup->setEnabled(FALSE);
    graspsGroup->setEnabled(FALSE);
    mDBMgr = graspItGUI->getIVmgr()->getDBMgr();
    if (mDBMgr) {
        getModelList();
    }
    sortBox->insertItem("Epsilon");
    sortBox->insertItem("Volume");
    sortBox->insertItem("Energy");
}

void DBaseDlg::destroy()
{
    //we do not delete the dbmgr because it is now set in the ivmgr for the rest of
    //graspit to use. The ivmgr deletes it on its exit.
    delete mModelScene;
}

void DBaseDlg::exitButton_clicked(){
    if (mCurrentLoadedModel) {
        //remove the previously loaded model, but don't delete it
        graspItGUI->getIVmgr()->getWorld()->destroyElement(mCurrentLoadedModel->getGraspableBody(), false);
    }
    //delete and release all the memories occupied by the grasps
    deleteVectorElements<db_planner::Grasp*, GraspitDBGrasp*>(mGraspList);
    mGraspList.clear();
    //delete and release all the memories occupied by the models
    deleteVectorElements<db_planner::Model*, GraspitDBModel*>(mModelList);
    QDialog::accept();
}

bool compareNames(db_planner::Model *m1, db_planner::Model *m2)
{
  return m1->ModelName() < m2->ModelName();
}

void DBaseDlg::getModelList()
{
    //clear the modelList
    deleteVectorElements<db_planner::Model*, GraspitDBModel*>(mModelList);
    mModelList.clear();
    //load the models from database manager
    if(!mDBMgr->ModelList(&mModelList,db_planner::FilterList::NONE)){
        DBGA("Model list retrieval failed");
        return;
    }
        //sort by names
        std::sort(mModelList.begin(), mModelList.end(), compareNames);
    //display the retrieved models, put their names into the combobox
    displayModelList();
    //check that there are valid number of models
    if(!mModelList.empty()){
        browserGroup->setEnabled(TRUE);
        connectButton->setEnabled(FALSE);
    }
    std::vector<std::string>graspTypes;
    if(!mDBMgr->GraspTypeList(&graspTypes)){
        DBGA("Grasp Types not loaded");
        return;
    }
    //display the types
    displayGraspTypeList(graspTypes);
}

/*! Deletes the old connection to the database and creates a new one based
    on the current settings in the dialog box. The new connection is then
    set as the one an only Database Manager that the rest of GraspIt had
    acces to.

    After connecting, it also reads the model list form the database and
    displays it.
*/
void DBaseDlg::connectButton_clicked()
{
    delete mDBMgr;
    Hand *h = graspItGUI->getIVmgr()->getWorld()->getCurrentHand();

#ifndef ROS_DATABASE_MANAGER
    mDBMgr = new db_planner::SqlDatabaseManager(hostLineEdit->text().toStdString(),
                            atoi(portLineEdit->text().latin1()),
                            usernameLineEdit->text().toStdString(),
                            passwordLineEdit->text().toStdString(),
                            databaseLineEdit->text().toStdString(),
                            new GraspitDBModelAllocator(),
                            new GraspitDBGraspAllocator(h));
#else
    mDBMgr = new db_planner::RosDatabaseManager(hostLineEdit->text().toStdString(),
                            portLineEdit->text().toStdString(),
                            usernameLineEdit->text().toStdString(),
                            passwordLineEdit->text().toStdString(),
                            databaseLineEdit->text().toStdString(),
                            new GraspitDBModelAllocator(),
                            new GraspitDBGraspAllocator(h));
        //use the special allocator for models that get geometry directly from the database
        GeomGraspitDBModelAllocator* allocator = new GeomGraspitDBModelAllocator(mDBMgr);
        mDBMgr->SetModelAllocator(allocator);
#endif

    if (mDBMgr->isConnected()) {
        getModelList();
    } else {
        DBGA("DBase Browser: Connection failed");
        delete mDBMgr;
        mDBMgr = NULL;
    }
    graspItGUI->getIVmgr()->setDBMgr(mDBMgr);
}


void DBaseDlg::loadGraspButton_clicked(){
    //get the current hand and check its validity
    Hand *hand = graspItGUI->getIVmgr()->getWorld()->getCurrentHand();
    if (!hand) {
        DBGA("Load and select a hand before viewing grasps!");
        return;
    }
    //check the currently loaded model
    if(!mCurrentLoadedModel){
        DBGA("Load model first!");
        return;
    }
    //clear the previously loaded grasps
    deleteVectorElements<db_planner::Grasp*, GraspitDBGrasp*>(mGraspList);
    mGraspList.clear();
    mCurrentFrame = 0;
    //get new grasps from database manager
    if(!mDBMgr->GetGrasps(*mCurrentLoadedModel,hand->getDBName().toStdString(), &mGraspList)){
        DBGA("Load grasps failed");
        mGraspList.clear();
        return;
    }
    for(std::vector<db_planner::Grasp*>::iterator it = mGraspList.begin(); it != mGraspList.end(); ){
        if( QString((*it)->GetSource().c_str()) == typesComboBox->currentText() ||
            typesComboBox->currentText() == "ALL") ++it;
        else{
            delete (*it);
            mGraspList.erase(it);
        }
    }
    //set corresponding indices and show the grasp
    QString numTotal, numCurrent;
    numTotal.setNum(mGraspList.size());
    if(!mGraspList.empty()){
        numCurrent.setNum(mCurrentFrame + 1);
        graspsGroup->setEnabled(TRUE);
        showGrasp(0);
        showMarkers();
    } else{
        numCurrent.setNum(0);
        graspsGroup->setEnabled(FALSE);
    }
    graspIndexLabel->setText(numCurrent + "/" + numTotal);
}

void DBaseDlg::loadModelButton_clicked(){
    if (mCurrentLoadedModel) {
        //remove the previously loaded model, but don't delete it
        graspItGUI->getIVmgr()->getWorld()->destroyElement(mCurrentLoadedModel->getGraspableBody(), false);
        mCurrentLoadedModel = NULL;
    }
    if(mModelList.empty()){
        DBGA("No model loaded...");
        return;
    }

    //check out the model in the modelList
    GraspitDBModel* model = dynamic_cast<GraspitDBModel*>(mModelList[mModelMap[modelsComboBox->currentText().toStdString()]]);
    if(!model){
        DBGA("Cannot recognize the model type");
        return;
    }
    //check that this model is already loaded into Graspit, if not, load it
    if (!model->geometryLoaded()) {
        //this loads the actual geometry in the scene graph of the object
        if ( model->load(graspItGUI->getIVmgr()->getWorld()) != SUCCESS) {
            DBGA("Model load failed");
            return;
        }
    }
    //adds the object to the collision detection system
    model->getGraspableBody()->addToIvc();
    //todo: where to dynamic information come from?
    //model->getGraspableBody()->initDynamics();
    //this adds the object to the graspit world so that we can see it
    graspItGUI->getIVmgr()->getWorld()->addBody(model->getGraspableBody());
    //and remember it
    mCurrentLoadedModel = model;
    //model->getGraspableBody()->showAxes(false);
    model->getGraspableBody()->setTransparency(0.0);
        model->getGraspableBody()->setTran(transf::IDENTITY);
    graspsGroup->setEnabled(FALSE);

    //delete the previously loaded grasps
    deleteVectorElements<db_planner::Grasp*, GraspitDBGrasp*>(mGraspList);
    mGraspList.clear();
    //initialize the grasp information for the new model
    initializeGraspInfo();
}

// go to see the next grasp
void DBaseDlg::nextGraspButton_clicked(){
    nextGrasp();
}

// go back to the previous grasp
void DBaseDlg::previousGraspButton_clicked(){
    previousGrasp();
}

// pop up the new window for the grasp planner
void DBaseDlg::plannerButton_clicked(){
    //check the existance of database manager
    if(!mDBMgr){
        DBGA("No dbase manager.");
        return;
    }
    //check the hand
    Hand *h = graspItGUI->getIVmgr()->getWorld()->getCurrentHand();
    if(!h){
        DBGA("No hand found currently");
        return;
    }
    //check the current model
    if(!mCurrentLoadedModel){
        DBGA("No object loaded");
        return;
    }
    //instantialize a new dialogue of type DBasePlannerDlg and pop it up
    DBasePlannerDlg *dlg = new DBasePlannerDlg(this, mDBMgr, mCurrentLoadedModel, h);
    dlg->setAttribute(Qt::WA_ShowModal, false);
    dlg->setAttribute(Qt::WA_DeleteOnClose, true);
    dlg->show();

    //delete the grasps loaded, release the memories, and reset the grasp information
    graspsGroup->setEnabled(FALSE);
    deleteVectorElements<db_planner::Grasp*, GraspitDBGrasp*>(mGraspList);
    initializeGraspInfo();
}

class VisualQualityFunctor {
public:
    enum Type{ENERGY, EPSILON, VOLUME};
    Type mType;
    double operator()(const db_planner::Grasp *grasp) {
        switch(mType) {
        case ENERGY:
            return grasp->Energy();
        case EPSILON:
            return grasp->EpsilonQuality();
        case VOLUME:
            return grasp->VolumeQuality();
        default:
            return 0.0;
        }
    }
};

void DBaseDlg::sortButton_clicked()
{
    if (mGraspList.empty()) return;
    VisualQualityFunctor func;
    if (sortBox->currentText()== "Energy") {
        std::sort(mGraspList.begin(), mGraspList.end(), db_planner::Grasp::CompareEnergy);
        func.mType = VisualQualityFunctor::ENERGY;
    } else if (sortBox->currentText()== "Epsilon") {
        std::sort(mGraspList.begin(), mGraspList.end(), db_planner::Grasp::CompareEpsilon);
        func.mType = VisualQualityFunctor::EPSILON;
    } else if (sortBox->currentText()== "Volume") {
        std::sort(mGraspList.begin(), mGraspList.end(), db_planner::Grasp::CompareVolume);
        func.mType = VisualQualityFunctor::VOLUME;
    } else {
        assert(0);
    }
    mCurrentFrame = 0;
    showGrasp(mCurrentFrame);
    if (showMarkersBox->isChecked()) {
        double min = func(mGraspList.front());
        double max = func(mGraspList.back());
        if (max==min) max=min+1.0;
        std::vector<db_planner::Grasp*>::iterator it;
        for (it=mGraspList.begin(); it!=mGraspList.end(); it++) {
            double r = (func(*it) - min) / (max - min);
            double g = 1-r;
            double b = 0.0;
            GraspPlanningState *state;
            if(showPreGraspRadioButton->isChecked()) {
                state = static_cast<GraspitDBGrasp*>(*it)->getPreGraspPlanningState();
            } else {
                state = static_cast<GraspitDBGrasp*>(*it)->getFinalGraspPlanningState();
            }
            state->setIVMarkerColor(r,g,b);
        }
    }
}

//a shortcut for the GWS display
void DBaseDlg::createGWSButton_clicked(){
    graspItGUI->getMainWindow()->graspCreateProjection();
}

//trigger when the selection in the model list combo box is changed, display the corresponding new image
void DBaseDlg::modelChanged(){
    if(inModelConstruction) return;
    QString psbModelThumbPath = QString( mModelList[mModelMap[modelsComboBox->currentText().toStdString()]]->
                         ThumbnailPath().c_str() );
    if(mModelScene) delete mModelScene;
    mModelScene = new QGraphicsScene;
    mModelScene->setBackgroundBrush(Qt::blue);
    QPixmap lPixmap;
    lPixmap.load(psbModelThumbPath);
    //resize so that it will fit in window
    if (lPixmap.width() > 160) {
        lPixmap = lPixmap.scaledToWidth(160);
    }
    if (lPixmap.height() > 120) {
        lPixmap = lPixmap.scaledToHeight(120);
    }
    mModelScene->addPixmap(lPixmap);
    this->objectGraph->setScene(mModelScene);
    this->objectGraph->show();
}

void DBaseDlg::showMarkers()
{
    std::vector<db_planner::Grasp*>::iterator it;
    for (it=mGraspList.begin(); it!=mGraspList.end(); it++) {
        GraspitDBGrasp *grasp = static_cast<GraspitDBGrasp*>(*it);
        if (showPreGraspRadioButton->isChecked()) {
            grasp->getFinalGraspPlanningState()->hideVisualMarker();
            if (showMarkersBox->isChecked()) {
                grasp->getPreGraspPlanningState()->showVisualMarker();
                //show markers in blue
                if (grasp->ClusterRep()) {
                  grasp->getPreGraspPlanningState()->setIVMarkerColor(0,0,1);
                } else if (grasp->CompliantCopy()) {
                  grasp->getPreGraspPlanningState()->setIVMarkerColor(1.0,1.0,0.0);
                                } else {
                  grasp->getPreGraspPlanningState()->setIVMarkerColor(0.5,0.5,0.5);
                }
            } else {
                grasp->getPreGraspPlanningState()->hideVisualMarker();
            }
        } else {
            grasp->getPreGraspPlanningState()->hideVisualMarker();
            if (showMarkersBox->isChecked()) {
                grasp->getFinalGraspPlanningState()->showVisualMarker();
                //show markers in blue
                if (grasp->ClusterRep()) {
                  grasp->getFinalGraspPlanningState()->setIVMarkerColor(0,0,1);
                } else if (grasp->CompliantCopy()) {
                  grasp->getFinalGraspPlanningState()->setIVMarkerColor(1.0,1.0,0.0);
                } else {
                  grasp->getFinalGraspPlanningState()->setIVMarkerColor(0.5,0.5,0.5);
                }
            } else {
                grasp->getFinalGraspPlanningState()->hideVisualMarker();
            }
        }
    }
}

//trigger when the grasp type is changed between pregrasp and final grasp
void DBaseDlg::graspTypeChanged(){
    showGrasp(mCurrentFrame);
    showMarkers();
}

//trigger when the model class is changed, reconstruct the model list combo box
void DBaseDlg::classChanged(){
    inModelConstruction = true;
    modelsComboBox->clear();
    for(size_t i = 0; i < mModelList.size(); ++i){
        if(mModelList[i]->Tags().find(classesComboBox->currentText().toStdString()) != mModelList[i]->Tags().end()
            || classesComboBox->currentText() == "ALL")
            modelsComboBox->addItem(mModelList[i]->ModelName().c_str());
    }
    inModelConstruction = false;
    modelChanged();
}

//synthesize the model list combo box
void DBaseDlg::displayModelList(){
    std::set<string> tags;
    mModelMap.clear();
    for(int i = 0; i < (int)mModelList.size(); ++i){
        modelsComboBox->insertItem(QString(mModelList[i]->ModelName().c_str()));
        tags.insert(mModelList[i]->Tags().begin(), mModelList[i]->Tags().end());
        mModelMap.insert(std::make_pair<std::string, int>(mModelList[i]->ModelName(), i));
    }
    classesComboBox->clear();
    classesComboBox->insertItem("ALL");
    for(std::set<string>::iterator i = tags.begin(); i != tags.end(); ++i){
        classesComboBox->insertItem(QString((*i).c_str()));
    }
}

void DBaseDlg::displayGraspTypeList(std::vector<std::string> list){
    typesComboBox->clear();
    typesComboBox->insertItem("ALL");
    for(size_t i = 0; i < list.size(); ++i){
        typesComboBox->insertItem(QString(list[i].c_str()));
    }
}

//core routine that shows the i-th loaded grasp
void DBaseDlg::showGrasp(int i)
{
    if (mGraspList.empty()) return;
    assert( i>=0 && i < (int)mGraspList.size() );
    //put the model in correct place
    mCurrentLoadedModel->getGraspableBody()->setTran(transf::IDENTITY);
    //show the pregrasp or final grasp
    if(showPreGraspRadioButton->isChecked()){
        if(!static_cast<GraspitDBGrasp*>(mGraspList[i])->getPreGraspPlanningState())//NULL grasp, return
            return;
        static_cast<GraspitDBGrasp*>(mGraspList[i])->getPreGraspPlanningState()->execute();
    }
    else{
        if(!static_cast<GraspitDBGrasp*>(mGraspList[i])->getFinalGraspPlanningState())//NULL grasp, return
            return;
        static_cast<GraspitDBGrasp*>(mGraspList[i])->getFinalGraspPlanningState()->execute();
        if(graspItGUI->getIVmgr()->getWorld()->getCurrentHand()->isA("Barrett")){
            graspItGUI->getIVmgr()->getWorld()->getCurrentHand()->autoGrasp(true);
        }
    }

    /*
    std::cerr << "Original tran: " << static_cast<GraspitDBGrasp*>(mGraspList[i])
      ->getFinalGraspPlanningState()->getPosition()->getCoreTran() << "\n";
    GraspPlanningState *state = new GraspPlanningState(static_cast<GraspitDBGrasp*>(mGraspList[i])
                               ->getFinalGraspPlanningState() );
    state->setPositionType(SPACE_AXIS_ANGLE, true);
    std::cerr << "     New tran: " << state->getPosition()->getCoreTran() << "\n";
    */

    //update the world and grasp information
    graspItGUI->getIVmgr()->getWorld()->findAllContacts();
    graspItGUI->getIVmgr()->getWorld()->updateGrasps();
    mCurrentFrame = i;
    updateGraspInfo();
}

//go to see the next grasp and show the corresponding image
void DBaseDlg::nextGrasp() {
    if (mGraspList.empty()) return;
    mCurrentFrame ++;
    if (mCurrentFrame == mGraspList.size()) mCurrentFrame = 0;
    showGrasp(mCurrentFrame);
}

//go to see the previous grasp and show the corresponding image
void DBaseDlg::previousGrasp() {
    if (mGraspList.empty()) return;
    mCurrentFrame --;
    if (mCurrentFrame < 0) mCurrentFrame = mGraspList.size() - 1;
    showGrasp(mCurrentFrame);
}

//update the information of current grasp, including indices, epsilon qualities, and volume qualities
void DBaseDlg::updateGraspInfo(){
    QString numTotal, numCurrent;
    numTotal.setNum(mGraspList.size());
    if(!mGraspList.empty())
        numCurrent.setNum(mCurrentFrame + 1);
    else
        numCurrent.setNum(0);
    graspIndexLabel->setText(numCurrent + "/" + numTotal);

    QString eq, vq, en, cl;
    eq.setNum(mGraspList[mCurrentFrame]->EpsilonQuality());
    vq.setNum(mGraspList[mCurrentFrame]->VolumeQuality());
    en.setNum(mGraspList[mCurrentFrame]->Energy());
    cl.setNum(mGraspList[mCurrentFrame]->Clearance());

    epsilonQualityLabel->setText(QString("Epsilon Quality: " + eq));
    volumeQualityLabel->setText(QString("Volume Quality: " + vq));
    energyLabel->setText(QString("Energy: " + en));
    clearanceLabel->setText(QString("Pregrasp clearance: " + cl));
}

//reset the grasp information displayed
void DBaseDlg::initializeGraspInfo(){
    graspIndexLabel->setText("0/0");
    epsilonQualityLabel->setText(QString("Epsilon Quality: 0.0"));
    volumeQualityLabel->setText(QString("Volume Quality: 0.0"));
}

//helper function that deletes the vector of type vectorType, but treating every elements as type treatAsType
template <class vectorType, class treatAsType>
inline void DBaseDlg::deleteVectorElements(std::vector<vectorType>& v){
    for(size_t i = 0; i < v.size(); ++i){
        delete (treatAsType)v[i];
    }
    v.clear();
}

// Comparator for sorting contacts + virtual contacts
struct contactComparator {
    static bool compare(vec3 v1, vec3 v2) {
        // sort x then z then y least to greatest
        if (v1.x() == v2.x()) {
            if (v1.z() == v2.z()) {
                return v1.y() < v2.y();
            } else {
                return v1.z() < v2.z();
            }
        } else {
            return v1.x() < v2.x();
        }
    }
};

void
DBaseDlg::saveBinvoxFromVoxVec(QString fileName, std::vector<bool> *voxVec) {

    Binvox *b = mCurrentLoadedModel->getBinvox();

    // Open file for output
    std::ofstream binvoxOutput(fileName.toStdString().c_str(), std::ios::out | std::ios::binary);

    if (binvoxOutput.is_open())
    {
        binvoxOutput << "#binvox 1\n";

        binvoxOutput << "dim " << b->depth << " " << b->height << " " << b->width << "\n";
        binvoxOutput << "translate " << b->tx << " " << b->ty << " " << b->tz << "\n";
        binvoxOutput << "scale " << b->scale << "\n";

        binvoxOutput << "data\n";

        bool value = 0;
        int count = 0;
        for (int i=0; i < voxVec->size(); i++) {
            if (voxVec->at(i)) {
                if (value && count == 255) {
                    count = 1;
                    binvoxOutput << (unsigned char)1 << (unsigned char)255;
                } else if (value) {
                    count++;
                } else if (!value) {
                    value = 1;
                    binvoxOutput << (unsigned char)0 << (unsigned char)count;
                    count = 1;
                }
            } else {
                if (!value && count == 255) {
                    count = 1;
                    binvoxOutput << (unsigned char)0 << (unsigned char)255;
                } else if (!value) {
                    count++;
                } else if (value) {
                    value = 0;
                    binvoxOutput << (unsigned char)1 << (unsigned char)count;
                    count = 1;
                }
            }
        }

        // remaining count
        if (!value) {
            binvoxOutput << (unsigned char)0 << (unsigned char)count;
        } else {
            binvoxOutput << (unsigned char)1 << (unsigned char)count;
        }

        // Done!
        binvoxOutput.close();

    } else {
        QTWARNING("could not open " + fileName + "for writing");
    }
}

void
DBaseDlg::saveBinvoxOfContacts(QString fileName, std::vector<vec3> contactLocs) {
    Binvox *b = mCurrentLoadedModel->getBinvox();
    double dbModelRescale = mCurrentLoadedModel->RescaleFactor();
    DBGA("b->scale, b->tx , b->ty , b->tz: " << b->scale <<"," << b->tx <<"," << b->ty << ","<< b->tz);

    //  Dump contacts to binvox
    std::vector<bool> voxVec;
    int y, z, x;
    double nym, nyM, nzm, nzM, nxm, nxM; // min and max dims of each binvox point
    int currentContactIndex = 0;

    // check if there's any contact in contactLocs list
    if (contactLocs.size() == 0){
        for (int i=0; i < b->size; i++)
            voxVec.push_back(0);

    }
    else {

        vec3 currentContact = contactLocs.at(currentContactIndex);
        vec3 *currentContactScaled = new vec3((currentContact.x()/dbModelRescale / b->scale - b->tx/ b->scale),
                                              (currentContact.y()/dbModelRescale / b->scale - b->ty/ b->scale),
                                              (currentContact.z()/dbModelRescale / b->scale - b->tz/ b->scale));
        //    vec3 *currentContactScaled = new vec3((currentContact.x()/dbModelRescale - b->tx) / b->scale,
        //                                          (currentContact.y()/dbModelRescale - b->ty) / b->scale,
        //                                          (currentContact.z()/dbModelRescale - b->tz) / b->scale);
        for (int i=0; i < b->size; i++){
            // int index = x * wxh + z * width + y;  // wxh = width * height = d * d
            // from http://www.cs.princeton.edu/~min/binvox/binvox.html
            // note: this might not work if w,h,d aren't all =
            x = i / b->width / b->height;
            z = (i % (b->width * b->height)) / b->depth;
            y = (i % (b->width * b->height)) % b->depth;

            //binvox normalizes coords to fit inside a 1.0x1.0x1.0 cube
            nym = (double)y / (double)b->depth;
            nzm = (double)z / (double)b->height;
            nxm = (double)x / (double)b->width;

            nyM = nym + (1.0 / (double)b->depth);
            nzM = nzm + (1.0 / (double)b->height);
            nxM = nxm + (1.0 / (double)b->width);

            if (i ==73267)
                int wait =50000;
            if ( (currentContactIndex < contactLocs.size()) && (
                     currentContactScaled->y() > 1.0 ||
                     currentContactScaled->x() > 1.0 ||
                     currentContactScaled->z() > 1.0)
                 ){
                DBGA("removing contact that is outside unit cube");
                DBGA("currentContactScaled  x, y, z: " << currentContactScaled->x() << " , " << currentContactScaled->y() << " , "<< currentContactScaled->z());
                delete currentContactScaled;
                currentContactIndex++;
                if (currentContactIndex == contactLocs.size()) {
                    continue;
                }

                currentContact = contactLocs.at(currentContactIndex);
                currentContactScaled = new vec3((currentContact.x()/dbModelRescale / b->scale - b->tx/ b->scale),
                                                (currentContact.y()/dbModelRescale / b->scale - b->ty/ b->scale),
                                                (currentContact.z()/dbModelRescale / b->scale - b->tz/ b->scale));

                //            currentContactScaled = new vec3((currentContact.x()/dbModelRescale - b->tx) / b->scale,
                //                                            (currentContact.y()/dbModelRescale - b->ty) / b->scale,
                //                                            (currentContact.z()/dbModelRescale - b->tz) / b->scale);
            }

            if ( (currentContactIndex < contactLocs.size()) &&
                 ((nym <= currentContactScaled->y()) && (currentContactScaled->y() <= nyM)) &&
                 ((nzm <= currentContactScaled->z()) && (currentContactScaled->z() <= nzM)) &&
                 ((nxm <= currentContactScaled->x()) && (currentContactScaled->x() <= nxM)) ) {

                voxVec.push_back(1);
                DBGA("Added! index: " << i <<" currentContactScaled  x, y, z: " << currentContactScaled->x() << " , " << currentContactScaled->y() << " , "<< currentContactScaled->z());


                delete currentContactScaled;
                currentContactIndex++;
                if (currentContactIndex == contactLocs.size()) {
                    continue;
                }

                currentContact = contactLocs.at(currentContactIndex);

                currentContactScaled = new vec3((currentContact.x()/dbModelRescale / b->scale - b->tx/ b->scale),
                                                (currentContact.y()/dbModelRescale / b->scale - b->ty/ b->scale),
                                                (currentContact.z()/dbModelRescale / b->scale - b->tz/ b->scale));
                //            currentContactScaled = new vec3((currentContact.x()/dbModelRescale + b->tx) * b->scale,
                //                                            (currentContact.y()/dbModelRescale + b->ty) * b->scale,
                //                                            (currentContact.z()/dbModelRescale + b->tz) * b->scale);


            } else {
                voxVec.push_back(0);
            }
        }
        if (currentContactIndex != contactLocs.size()){
            DBGA("voxvec did not finish properly..." << currentContactIndex << " , " << contactLocs.size());
            DBGA("currentContactScaled  x, y, z: " << currentContactScaled->x() << " , " << currentContactScaled->y() << " , "<< currentContactScaled->z());
        }
    }

    saveBinvoxFromVoxVec(fileName, &voxVec);
}

/*! Direct computes the index locations of the finger tip position in the binvox files
 * Returns the vector of binvox vector
*/
void
DBaseDlg::saveBinvoxOfContactsDirectIndex(QString fileName, std::vector<vec3> contactLocs) {
    Binvox *b = mCurrentLoadedModel->getBinvox();
    double dbModelRescale = mCurrentLoadedModel->RescaleFactor();
    DBGA("b->scale, b->tx , b->ty , b->tz: " << b->scale <<"," << b->tx <<"," << b->ty << ","<< b->tz);

    //  Dump contacts to binvox
    std::vector<bool> voxVec;

    for (int i=0; i < b->size; i++)
        voxVec.push_back(0);

    for (int j = 0; j < contactLocs.size(); j++){
        vec3 currentContact = contactLocs.at(j);
        vec3 *currentContactScaled = new vec3((currentContact.x()/dbModelRescale / b->scale - b->tx),
                                          (currentContact.y()/dbModelRescale / b->scale - b->ty),
                                          (currentContact.z()/dbModelRescale / b->scale - b->tz));
//        int vox_index = (currentContactScaled[1] + currentContactScaled[2] * b->depth  + currentContactScaled[0] * b->depth * b->width) ;
        int vox_index = (currentContactScaled->y() + currentContactScaled->z() * b->depth  + currentContactScaled->z() * b->depth * b->width) ;

        DBGA("Invalid contact location; exceeds binvox box"<< vox_index << " , " << voxVec.size());
        if (vox_index < voxVec.size())
            voxVec[vox_index] = 1;
        else
            DBGA("Invalid contact location; exceeds binvox box");
    }

    saveBinvoxFromVoxVec(fileName, &voxVec);
}

/*! Gets the current contact point location of the fingers on the object.
 * Return the list of the locations in the object's frame of refrence
*/
std::vector<vec3>
DBaseDlg::getContactPointsLocationsFromHand() {

    World *w = graspItGUI->getIVmgr()->getWorld();

    // Get contacts
    transf bodyInWorld = w->getGB(0)->getTran();

    w->getHand(0)->getGrasp()->collectContacts();
    double cPos[3];
    std::vector<vec3> contactLocations;
    for (int i=0; i<w->getHand(0)->getGrasp()->getNumContacts(); i++) {
        Contact *c = w->getHand(0)->getGrasp()->getContact(i);
        c->getLocation().get(cPos);
        assert(false);
        // Need to use getWorldLocation otherwise you're just
        // returning the location in the frame of ref in the world
        // and the following transformation doesn't do anything
        transf contactInWorld(Quaternion::IDENTITY, vec3(cPos));
        transf contactInBody = bodyInWorld.inverse() * contactInWorld;
        contactLocations.push_back(contactInBody.translation());
        // Uncomment the following lines to see the placement of contacts as spheres in the scene
        //Body * tempBody = w->importBody("Body","/home/iakinola/graspit/models/objects/sphere.xml");
        //tempBody->setTran(contactInBody);
    }
    std::sort(contactLocations.begin(), contactLocations.end(), contactComparator::compare);
    return contactLocations;
}



/*! Gets the current virtual contact point location of the fingers on the object.
 * Return the list of the locations in the object's frame of refrence
*/
std::vector<vec3>
DBaseDlg::getVirtualContactPointsLocationsFromHand() {

    World *w = graspItGUI->getIVmgr()->getWorld();

    transf bodyInWorld = w->getGB(0)->getTran();
    w->getHand(0)->getGrasp()->collectVirtualContacts();
    VirtualContact *vc;
    double vcPos[3];
    std::vector<vec3> vcLocations;
    for (int i=0; i<w->getHand(0)->getGrasp()->getNumContacts(); i++) {
        vc = (VirtualContact*)w->getHand(0)->getGrasp()->getContact(i);
        vc->getWorldLocation().get(vcPos);
        transf contactInWorld(Quaternion::IDENTITY, vec3(vcPos));
        transf contactInBody = bodyInWorld.inverse() * contactInWorld;
        vcLocations.push_back(contactInBody.translation());
        // Uncomment the following lines to see the placement of contacts as spheres in the scene
        //Body * tempBody = w->importBody("Body","/home/iakinola/graspit/models/objects/sphere.xml");
        //tempBody->setTran(contactInBody);
    }
    std::sort(vcLocations.begin(), vcLocations.end(), contactComparator::compare);
    DBGA("Number of contact locations. \t" << vcLocations.size());
    return vcLocations;
}



/*! Gets the current contact point location of the fingers on the object.
 * Return the list of the locations in the object's frame of refrence
*/
std::vector<vec3>
DBaseDlg::getContactPointsLocationsFromObject() {

    World *w = graspItGUI->getIVmgr()->getWorld();

    // this portion deals directly with the object.
    w->getHand(0)->getGrasp()->collectContacts();
    std::list<Contact*> contactList = w->getGB(0)->getContacts();       // get the list of Contacts on the object

    DBGA("There are  " << contactList.size() << " contacts in contactList.... Ireti debugging");

    double vcPos[3];
    std::vector<vec3> contactLocations;
    std::list<Contact*>::const_iterator it;

    for (it = contactList.begin(); it!=contactList.end(); it++) {
            (*it)->getLocation().get(vcPos);
            contactLocations.push_back(vec3 (vcPos));

            DBGA("Contact location-- x: " << vcPos[0] << ", y: " << vcPos[1] << ", z: " << vcPos[2] << " contacts in contactList.... Ireti debugging");
            //Uncomment the following lines to see the placement of contacts as spheres in the scene
            //Body * tempBody = w->importBody("Body","/home/iakinola/graspit/models/objects/sphere.xml");
            //transf contactTran(Quaternion::IDENTITY, vcPos);
            //tempBody->setTran(contactTran);
    }

    if (contactLocations.size()!=0)
        std::sort(contactLocations.begin(), contactLocations.end(), contactComparator::compare);
    else
        DBGA("There are not contact points in this instance" << w->getGB(0)->getFilename().toStdString());
    return contactLocations;
}


/*! Saves the current hand configuration in a binvox with dimensions
matching the currently loaded graspable body.
Note: This function can only be run when there is one robot (a hand)
in the scene and only on graspable body that also must have a binvox
member variable loaded.
*/
void
DBaseDlg::saveBinvoxButton_clicked() {

    World *w = graspItGUI->getIVmgr()->getWorld();

    if (w->getNumHands() != 1) {
        QTWARNING("saveHandVox: There can only be one hand loaded in the world when exporting a binvox.");
        return;
    } else if (w->getNumGB() != 1) {
        QTWARNING("saveHandVox: There can only be one graspable body loaded in the world when exporting a binvox.");
        return;
    } else if (!mCurrentLoadedModel->binvoxLoaded()) {
        QTWARNING("saveHandVox: Body does not have a .binvox loaded.");
        return;
    } else if (mCurrentLoadedModel->getBinvox()->version != 1) {
        QTWARNING("saveHandVox: Loaded .binvox is not version 1. We only support version 1 of binvox.");
        return;
    }


    QString fileName ="";
    QString fn = QFileDialog::getSaveFileName(this, QString(), QString(getenv("GRASPIT")),
                                              "Binvox Files (*.binvox)" );
    if ( !fn.isEmpty() ) {
      fileName = fn;
      if (fileName.section('.',1).isEmpty()) {
        fileName.append(".binvox");
      }

    } else {
        QTWARNING("Error saving binvox... you didn't name it properly...");
        return;
    }

//    saveBinvoxOfContacts(fileName, getVirtualContactPointsLocationsFromHand());
    saveBinvoxOfContacts(fileName, getContactPointsLocationsFromObject());
}


/*! re-evaluates the grasp quality on the grasp loaded from database
*/
energyValues DBaseDlg::evaluateGraspInDB(int graspIndexInDB)
{
    Hand *mHand = graspItGUI->getIVmgr()->getWorld()->getCurrentHand();
    GraspPlanningState gps = static_cast<GraspitDBGrasp*>(mGraspList[graspIndexInDB])->getFinalGraspPlanningState();
    gps.execute(mHand);
//    gps.getEnergy();//    gps.getVolume();//            gps.getEpsilonQuality();


    mHand->autoGrasp(true);
    //    mHand->autoGrasp(render_it, 1.0, false);

    SearchEnergy *mEnergyCalculator = new SearchEnergy();
    mEnergyCalculator->setType(ENERGY_CONTACT_QUALITY);  //ENERGY_POTENTIAL_QUALITY
    mEnergyCalculator->setContactType(CONTACT_PRESET); //CONTACT_LIVE, CONTACT_PRESET

    bool is_legal;
    double new_planned_energy;

    mEnergyCalculator->analyzeCurrentPosture(mHand,graspItGUI->getMainWorld()->getGB(0),is_legal,new_planned_energy,false );
    //    gps.setEnergy(new_planned_energy);

    QualEpsilon *mQualEpsilon;
    mQualEpsilon = new QualEpsilon(mHand->getGrasp(), QString("Grasp_recorder_qm"), "L1 Norm");
    double quality = mQualEpsilon->evaluate();

    QualVolume *mQualVolume;
    mQualVolume = new QualVolume(mHand->getGrasp(), QString("Grasp_recorder_qm"), "L1 Norm");

    energyValues ev = {new_planned_energy,
                        mQualEpsilon->evaluate(),
                        mQualVolume->evaluate(),
                        gps.getEpsilonQuality(),
                        gps.getVolume()};
    delete mQualEpsilon;
    delete mQualVolume;

    return ev;
/*
    DBGA("Calculated grasp: new_planned_energy \t" <<  new_planned_energy<< " , mQualEpsilon \t"
         << mQualEpsilon->evaluate() << " , mQualVolume \t" <<mQualVolume->evaluate());

    return new_planned_energy;*/
}

/*! re-evaluates the grasp quality on the grasp loaded from database
*/
energyValues DBaseDlg::perturbAndEvaluateGraspInDB(int graspIndexInDB)
{
    bool render_it = true;

    Hand *mHand = graspItGUI->getIVmgr()->getWorld()->getCurrentHand();
    GraspPlanningState gps = static_cast<GraspitDBGrasp*>(mGraspList[graspIndexInDB])->getFinalGraspPlanningState();
    gps.setObject(graspItGUI->getMainWorld()->getGB(0));

//    GraspableBody *objectBody = graspItGUI->getMainWorld()->getGB(0);
    //objectBody->getIVScaleTran();
//    objectBody->getPos();


    //disable collisions
    graspItGUI->getMainWorld()->toggleAllCollisions(false);
    usleep(10000);
    gps.execute(mHand);

    if(render_it)           //visualize initial state
    {
        graspItGUI->getIVmgr()->getViewer()->render();
        usleep(1000000);
    }

    //mHand->quickOpen();
    mHand->autoGrasp(true, -10.0, false);       // open hand
    if(render_it)
    {
        graspItGUI->getIVmgr()->getViewer()->render();
        usleep(1000000);
    }

    mHand->approachToContact(-200.0);           // withdraw hand from object
    if(render_it)
    {
        graspItGUI->getIVmgr()->getViewer()->render();
        usleep(1000000);
    }


    // shift the hand a bit to create another grasp angle
    double max_displacement = -50.;
    double deltax = ((double) rand() / (RAND_MAX)) * max_displacement/2;
    double deltay = ((double) rand() / (RAND_MAX)) * max_displacement/2;
    double deltaz = ((double) rand() / (RAND_MAX)) * max_displacement*2;
    DBGA("random displacements \t" << deltax << " , " << deltay << " , " << deltaz);
    transf newTran = translate_transf(vec3(deltax,deltay,deltaz) * mHand->getApproachTran()) * mHand->getTran();

    // move the hand to a new location
    //    moveTo(transf &newTran,double translStepSize, double rotStepSize);
    mHand->moveTo(newTran, WorldElement::ONE_STEP, WorldElement::ONE_STEP);      //(transf &newTran,double translStepSize, double rotStepSize)
    if(render_it)
    {
        graspItGUI->getIVmgr()->getViewer()->render();
        usleep(1000000);
    }

    graspItGUI->getMainWorld()->toggleAllCollisions(true);      // activate collisions
    double dist = mHand->getApproachDistance(graspItGUI->getMainWorld()->getGB(0), 300);
    DBGA("New dustance to object: \t" << dist);
    //dist = 200
    mHand->approachToContact(dist);                              // move to object until collision
    if(render_it)
    {
        graspItGUI->getIVmgr()->getViewer()->render();
        usleep(1000000);
    }

    //gps.execute(mHand);
    //possible
    //mHand->moveDOFToContacts();
    //mHand->checkSetDOFVals();
    //moveDOFToContacts
    //mHand->jumpDOFToContact();
    //mHand->moveDOFToContacts(double *desiredVals, double *desiredSteps, bool stopAtContact, bool renderIt);

    if(render_it)
    {
        graspItGUI->getIVmgr()->getViewer()->render();
        usleep(1000000);
    }
    mHand->autoGrasp(render_it, 1.0, false);                    // grasp object

    if(render_it)
    {
        graspItGUI->getIVmgr()->getViewer()->render();
        usleep(1000000);
    }


    // Analyse grasp
    SearchEnergy *mEnergyCalculator = new SearchEnergy();
    mEnergyCalculator->setType(ENERGY_CONTACT_QUALITY);  //ENERGY_POTENTIAL_QUALITY
    mEnergyCalculator->setContactType(CONTACT_PRESET); //CONTACT_LIVE, CONTACT_PRESET

    bool is_legal;
    double new_planned_energy;
    mEnergyCalculator->analyzeCurrentPosture(mHand,graspItGUI->getMainWorld()->getGB(0),is_legal,new_planned_energy,false );
    gps.setEnergy(new_planned_energy);

    QualEpsilon *mQualEpsilon;
    mQualEpsilon = new QualEpsilon(mHand->getGrasp(), QString("Grasp_recorder_qm"), "L1 Norm");
    double epsilonQuality = mQualEpsilon->evaluate();

    QualVolume *mQualVolume;
    mQualVolume = new QualVolume(mHand->getGrasp(), QString("Grasp_recorder_qm"), "L1 Norm");
    double volumeQuality = mQualVolume->evaluate();

//    gps.setEnergy();


    DBGA("Calculated grasp: new_planned_energy \t" <<  new_planned_energy
         << " , mQualEpsilon \t" << mQualEpsilon->evaluate()
         << " , mQualVolume \t" <<mQualVolume->evaluate()
         << " , gps.getEpsilonQuality() \t" << gps.getEpsilonQuality()
         << " , gps.getVolume() \t" <<gps.getVolume());

    energyValues ev = {new_planned_energy,
                        mQualEpsilon->evaluate(),
                        mQualVolume->evaluate(),
                        gps.getEpsilonQuality(),
                        gps.getVolume()};

    delete mQualEpsilon;
    delete mQualVolume;

    return ev;
}







/*! Saved .binvox representations for all grasps in the
CGDB (for current category). Uses only objects where scale=1.0 in order to preserve
the transformation between the fingertips and the object (since we only
have the voxelized representation at scale 1.0)
*/
void
DBaseDlg::runBatchBinvoxButton_clicked() {

    for (int i=0; i < modelsComboBox->count(); i++) {
        // load the model
        modelsComboBox->setCurrentIndex(i);
        if (modelsComboBox->currentText().toStdString().substr(4,3) == "1.0") {
            DBGA("current object: " << modelsComboBox->currentText().toStdString());
        } else {
            continue; // skip this model if it's not at the right scale...
        }

        loadModelButton_clicked();

        // load the (EIGENGRASPS) grasQps
        for (int t=0; t<typesComboBox->count(); t++) {
            typesComboBox->setCurrentIndex(t);
            if (typesComboBox->currentText().toStdString() == "EIGENGRASPS")
                break;
        }
        loadGraspButton_clicked();


        // create directory to save the files corresponding to the current model.
        QString path(QString(getenv("GRASPIT")) + QString("/binvoxes/") + QString(modelsComboBox->currentText()));
        QDir dir(path.toStdString().c_str());
        if (!dir.exists())
            dir.mkpath(".");

        // store location of the original binvox location
        QString binvoxLocFilename(path + QString("/") + path.section('/',-1,-1) + QString(".location"));
        QString binvoxLocation = QString(mCurrentLoadedModel->GeometryPath().c_str());
        binvoxLocation = binvoxLocation.left(binvoxLocation.findRev('.')) +".binvox";

        DBGA(" binvoxLocation " << binvoxLocation.toStdString()<< " .... Ireti debugging");
        DBGA("binvoxLocFilename " << binvoxLocFilename.toStdString()<< " .... Ireti debugging");

        std::ofstream binvoxLocFileOutput(binvoxLocFilename.toStdString().c_str(), std::ios::out | std::ios::binary);
        if (binvoxLocFileOutput.is_open())
        {
            binvoxLocFileOutput << binvoxLocation.toStdString() << "\n";
        } else {
            QTWARNING("could not open " + binvoxLocFilename + "for writing");
        }
        binvoxLocFileOutput.close();

        // go through the grasps for this model and save
        for (int g=0; g<mGraspList.size(); g++) {

            QString vcFilename(path + QString("/") + QString::number(g+1) + QString("_vc"));
            QString binvoxVCFilename(vcFilename + QString(".binvox"));
            QString qualityVCFilename(vcFilename + QString(".quality"));
            //saveBinvoxOfContacts(binvoxCFilename, getContactPointsLocationsFromHand());
//            saveBinvoxOfContacts(binvoxVCFilename, getVirtualContactPointsLocationsFromHand());

            saveBinvoxOfContacts(binvoxVCFilename, getContactPointsLocationsFromObject());

            // also save the energy!
/*            std::ofstream qualityOutput(qualityVCFilename.toStdString().c_str(), std::ios::out | std::ios::binary);

            if (qualityOutput.is_open())
            {
                qualityOutput << "epsilon_quality=" << mGraspList[g]->EpsilonQuality() << "\n";
                qualityOutput << "volume_quality=" << mGraspList[g]->VolumeQuality() << "\n";
                qualityOutput.close();
            } else {
                QTWARNING("could not open " + qualityVCFilename + "for writing");
            }
            qualityOutput.close();*/

            // Re-evaluate Grasp Here

            energyValues ev = evaluateGraspInDB(g);

            std::ofstream qualityOutput(qualityVCFilename.toStdString().c_str(), std::ios::out | std::ios::binary);
            if (qualityOutput.is_open())
            {
                qualityOutput << "gpsEpsilon=" << ev.gpsEpsilon << "\n";
                qualityOutput << "gpsVolume=" << ev.gpsVolume << "\n";
                qualityOutput << "mQualEpsilon=" << ev.mQualEpsilon << "\n";
                qualityOutput << "mQualVolume=" << ev.mQualVolume << "\n";
                qualityOutput << "new_planned_energy=" << ev.new_planned_energy << "\n";

                qualityOutput.close();
            } else {
                QTWARNING("could not open " + qualityVCFilename + "for writing");
            }
            qualityOutput.close();

            for (int j=0; j<5; j++) {
                energyValues ev = perturbAndEvaluateGraspInDB(g);

                QString perturbedIndexString;
                std::ostringstream convert;
                convert << j;
                perturbedIndexString = QString(convert.str().c_str());

                DBGA("Warning: saving binvox file for perturbed grasp");
                QString perturbedGraspBinvoxFilename(vcFilename + QString("_") + perturbedIndexString + QString(".binvox"));
                saveBinvoxOfContacts(perturbedGraspBinvoxFilename, getContactPointsLocationsFromObject());

                QString perturbedQualityVCFilename(vcFilename + QString("_") + perturbedIndexString + QString(".quality"));
                std::ofstream preturbedGraspQualOut(perturbedQualityVCFilename.toStdString().c_str(), std::ios::out | std::ios::binary);

                if (preturbedGraspQualOut.is_open())
                {
                    preturbedGraspQualOut << "gpsEpsilon=" << ev.gpsEpsilon << "\n";
                    preturbedGraspQualOut << "gpsVolume=" << ev.gpsVolume << "\n";
                    preturbedGraspQualOut << "mQualEpsilon=" << ev.mQualEpsilon << "\n";
                    preturbedGraspQualOut << "mQualVolume=" << ev.mQualVolume << "\n";
                    preturbedGraspQualOut << "new_planned_energy=" << ev.new_planned_energy << "\n";

                    preturbedGraspQualOut.close();
                } else {
                    QTWARNING("could not open perurbed grasp quality output file for writing");
                }
                preturbedGraspQualOut.close();

            }

            nextGraspButton_clicked(); //use this button to load the next grasp to save
        }
    }
}
