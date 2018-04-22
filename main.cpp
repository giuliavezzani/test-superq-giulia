/******************************************************************************
 *                                                                            *
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @file main.cpp
 * @authors: Ugo Pattacini <ugo.pattacini@iit.it>
 */

#include <cstdlib>
#include <memory>
#include <cmath>
#include <vector>
#include <set>
#include <algorithm>
#include <string>
#include <sstream>
#include <fstream>

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>
#include <yarp/math/Rand.h>

#include <vtkSmartPointer.h>
#include <vtkProperty.h>
#include <vtkPolyDataMapper.h>
#include <vtkPointData.h>
#include <vtkSuperquadric.h>
#include <vtkUnsignedCharArray.h>
#include <vtkTransform.h>
#include <vtkSampleFunction.h>
#include <vtkContourFilter.h>
#include <vtkRadiusOutlierRemoval.h>
#include <vtkActor.h>
#include <vtkOrientationMarkerWidget.h>
#include <vtkAxesActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkCamera.h>
#include <vtkInteractorStyleSwitch.h>

#include "nlp.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;

/****************************************************************/
class Object
{
protected:
    vtkSmartPointer<vtkPolyDataMapper> vtk_mapper;
    vtkSmartPointer<vtkActor> vtk_actor;

public:
    /****************************************************************/
    vtkSmartPointer<vtkActor> &get_actor()
    {
        return vtk_actor;
    }
};


/****************************************************************/
class Points : public Object
{
protected:
    vtkSmartPointer<vtkPoints> vtk_points;
    vtkSmartPointer<vtkUnsignedCharArray> vtk_colors;
    vtkSmartPointer<vtkPolyData> vtk_polydata;
    vtkSmartPointer<vtkVertexGlyphFilter> vtk_glyphFilter;

public:
    /****************************************************************/
    Points(const vector<Vector> &points, const int point_size)
    {
        vtk_points=vtkSmartPointer<vtkPoints>::New();
        for (size_t i=0; i<points.size(); i++)
            vtk_points->InsertNextPoint(points[i][0],points[i][1],points[i][2]);

        vtk_polydata=vtkSmartPointer<vtkPolyData>::New();
        vtk_polydata->SetPoints(vtk_points);

        vtk_glyphFilter=vtkSmartPointer<vtkVertexGlyphFilter>::New();
        vtk_glyphFilter->SetInputData(vtk_polydata);
        vtk_glyphFilter->Update();

        vtk_mapper=vtkSmartPointer<vtkPolyDataMapper>::New();
        vtk_mapper->SetInputConnection(vtk_glyphFilter->GetOutputPort());

        vtk_actor=vtkSmartPointer<vtkActor>::New();
        vtk_actor->SetMapper(vtk_mapper);
        vtk_actor->GetProperty()->SetPointSize(point_size);
    }

    /****************************************************************/
    void set_points(const vector<Vector> &points)
    {
        vtk_points=vtkSmartPointer<vtkPoints>::New();
        for (size_t i=0; i<points.size(); i++)
            vtk_points->InsertNextPoint(points[i][0],points[i][1],points[i][2]);

        vtk_polydata->SetPoints(vtk_points);
    }

    /****************************************************************/
    bool set_colors(const vector<vector<unsigned char>> &colors)
    {
        if (colors.size()==vtk_points->GetNumberOfPoints())
        {
            vtk_colors=vtkSmartPointer<vtkUnsignedCharArray>::New();
            vtk_colors->SetNumberOfComponents(3);
            for (size_t i=0; i<colors.size(); i++)
                vtk_colors->InsertNextTypedTuple(colors[i].data());

            vtk_polydata->GetPointData()->SetScalars(vtk_colors);
            return true;
        }
        else
            return false;
    }

    /****************************************************************/
    vtkSmartPointer<vtkPolyData> &get_polydata()
    {
        return vtk_polydata;
    }
};


/****************************************************************/
class Superquadric : public Object
{
protected:
    vtkSmartPointer<vtkSuperquadric> vtk_superquadric;
    vtkSmartPointer<vtkSampleFunction> vtk_sample;
    vtkSmartPointer<vtkContourFilter> vtk_contours;
    vtkSmartPointer<vtkTransform> vtk_transform;

public:
    /****************************************************************/
    Superquadric(const Vector &r, const double color)
    {
        double bx=2.0*r[7];
        double by=2.0*r[8];
        double bz=2.0*r[9];

        vtk_superquadric=vtkSmartPointer<vtkSuperquadric>::New();
        vtk_superquadric->ToroidalOff();
        vtk_superquadric->SetSize(1.0);
        vtk_superquadric->SetCenter(zeros(3).data());

        vtk_superquadric->SetScale(r[7],r[8],r[9]);
        vtk_superquadric->SetPhiRoundness(r[10]);
        vtk_superquadric->SetThetaRoundness(r[11]);

        vtk_sample=vtkSmartPointer<vtkSampleFunction>::New();
        vtk_sample->SetSampleDimensions(50,50,50);
        vtk_sample->SetImplicitFunction(vtk_superquadric);
        vtk_sample->SetModelBounds(-bx,bx,-by,by,-bz,bz);

        vtk_contours=vtkSmartPointer<vtkContourFilter>::New();
        vtk_contours->SetInputConnection(vtk_sample->GetOutputPort());
        vtk_contours->GenerateValues(1,1.0,1.0);

        vtk_mapper=vtkSmartPointer<vtkPolyDataMapper>::New();
        vtk_mapper->SetInputConnection(vtk_contours->GetOutputPort());
        vtk_mapper->SetScalarRange(0.0,color);

        vtk_actor=vtkSmartPointer<vtkActor>::New();
        vtk_actor->SetMapper(vtk_mapper);
        vtk_actor->GetProperty()->SetOpacity(0.25);

        vtk_transform=vtkSmartPointer<vtkTransform>::New();
        vtk_transform->Translate(r.subVector(0,2).data());
        vtk_transform->RotateWXYZ((180.0/M_PI)*r[6],r.subVector(3,5).data());
        vtk_actor->SetUserTransform(vtk_transform);
    }
};


/****************************************************************/
class Finder : public RFModule
{
    Bottle outliersRemovalOptions;
    unsigned int sample;
    bool test_derivative;
    double inside_penalty;

    vector<Vector> all_points,in_points,out_points,dwn_points;
    vector<vector<unsigned char>> all_colors;
    
    unique_ptr<Points> vtk_all_points,vtk_out_points,vtk_dwn_points;
    
    BufferedPort<Bottle> oPort;
    BufferedPort<Property> iPort;

    RpcClient superqRpc;

    /****************************************************************/
    void removeOutliers()
    {
        if (outliersRemovalOptions.size()>=2)
        {
            double radius=outliersRemovalOptions.get(0).asDouble();
            int neighbors=outliersRemovalOptions.get(1).asInt();

            vtkSmartPointer<vtkPoints> vtk_points=vtkSmartPointer<vtkPoints>::New();
            for (size_t i=0; i<all_points.size(); i++)
                vtk_points->InsertNextPoint(all_points[i][0],all_points[i][1],all_points[i][2]);

            vtkSmartPointer<vtkPolyData> vtk_polydata=vtkSmartPointer<vtkPolyData>::New();
            vtk_polydata->SetPoints(vtk_points);

            vtkSmartPointer<vtkRadiusOutlierRemoval> removal=vtkSmartPointer<vtkRadiusOutlierRemoval>::New();
            removal->SetInputData(vtk_polydata);
            removal->SetRadius(radius);
            removal->SetNumberOfNeighbors(neighbors);
            removal->Update();

            yInfo()<<"# of outliers removed / # of points ="
                   <<removal->GetNumberOfPointsRemoved()<<"/"<<all_points.size();

            for (size_t i=0; i<all_points.size(); i++)
            {
                if (removal->GetPointMap()[i]<0)
                    out_points.push_back(all_points[i]);
                else
                    in_points.push_back(all_points[i]);
            }
        }
        else
            in_points=all_points;
    }

    /****************************************************************/
    void sampleInliers()
    {
        set<unsigned int> idx;
        while (idx.size()<sample)
        {
            unsigned int i=(unsigned int)(Rand::scalar(0.0,1.0)*in_points.size());
            if (idx.find(i)==idx.end())
            {
                dwn_points.push_back(in_points[i]);
                idx.insert(i);
            }
        }

        yInfo()<<"# of sampled inliers ="<<dwn_points.size();
    }

    /****************************************************************/
    Vector findSuperquadric() const
    {
        Ipopt::SmartPtr<Ipopt::IpoptApplication> app=new Ipopt::IpoptApplication;
        app->Options()->SetNumericValue("tol",1e-6);
        app->Options()->SetIntegerValue("acceptable_iter",0);
        app->Options()->SetStringValue("mu_strategy","adaptive");
        app->Options()->SetIntegerValue("max_iter",1000);
        app->Options()->SetStringValue("hessian_approximation","limited-memory");
        app->Options()->SetStringValue("derivative_test",test_derivative?"first-order":"none");
        app->Options()->SetIntegerValue("print_level",test_derivative?5:0);
        app->Initialize();

        double t0=Time::now();
        Ipopt::SmartPtr<SuperQuadricNLP> nlp=new SuperQuadricNLP(dwn_points,inside_penalty);
        Ipopt::ApplicationReturnStatus status=app->OptimizeTNLP(GetRawPtr(nlp));
        double t1=Time::now();

        Vector r=nlp->get_result();
        yInfo()<<"center   = ("<<r.subVector(0,2).toString(3,3)<<")";
        yInfo()<<"angle    ="<<r[3]<<"[deg]";
        yInfo()<<"size     = ("<<r.subVector(4,6).toString(3,3)<<")";
        yInfo()<<"shape    = ("<<r.subVector(7,8).toString(3,3)<<")";
        yInfo()<<"found in ="<<t1-t0<<"[s]";

        Vector rot(4,0.0);
        rot[2]=1.0; rot[3]=(M_PI/180.0)*r[3];

        Vector r_=r.subVector(0,2);
        r_=cat(r_,rot);
        r_=cat(r_,r.subVector(4,6));
        r_=cat(r_,r.subVector(7,8));

        return r_;
    }

    /****************************************************************/
    Vector  getBottle(Bottle &estimated_superq)
    {
        Vector superq_aux(12,0.0);
        Bottle *all=estimated_superq.get(0).asList();

        for (size_t i=0; i<all->size(); i++)
        {
            Bottle *group=all->get(i).asList();
            if (group->get(0).asString() == "dimensions")
            {
                 Bottle *dim=group->get(1).asList();

                 superq_aux[0]=dim->get(0).asDouble(); superq_aux[1]=dim->get(1).asDouble(); superq_aux[2]=dim->get(2).asDouble();
            }
            else if (group->get(0).asString() == "exponents")
            {
                 Bottle *dim=group->get(1).asList();

                 superq_aux[3]=dim->get(0).asDouble(); superq_aux[4]=dim->get(1).asDouble();
            }
            else if (group->get(0).asString() == "center")
            {
                 Bottle *dim=group->get(1).asList();

                 superq_aux[5]=dim->get(0).asDouble(); superq_aux[6]=dim->get(1).asDouble(); superq_aux[7]=dim->get(2).asDouble();
            }
            else if (group->get(0).asString() == "orientation")
            {


                 Bottle *dim=group->get(1).asList();

                 superq_aux[8]=dim->get(0).asDouble(); superq_aux[9]=dim->get(1).asDouble(); superq_aux[10]=dim->get(2).asDouble(); superq_aux[11]=dim->get(3).asDouble();
            }

        }


        return superq_aux;
    }

    /****************************************************************/
    bool configure(ResourceFinder &rf) override
    {
        Rand::init();

        bool streaming_mode=false;
        if (rf.check("file"))
        {
            string file=rf.find("file").asString();
            ifstream fin(file.c_str());
            if (!fin.is_open())
            {
                yError()<<"Unable to open file \""<<file<<"\"";
                return false;
            }

            Vector p(3);
            vector<unsigned int> c_(3);
            vector<unsigned char> c(3);

            string line;
            while (getline(fin,line))
            {
                istringstream iss(line);
                if (!(iss>>p[0]>>p[1]>>p[2]))
                    break;
                all_points.push_back(p);

                fill(c_.begin(),c_.end(),120);
                iss>>c_[0]>>c_[1]>>c_[2];
                c[0]=(unsigned char)c_[0];
                c[1]=(unsigned char)c_[1];
                c[2]=(unsigned char)c_[2];
                all_colors.push_back(c);
            }
        }
        else
        {
            yError()<<"Unable to find command-line option \"--file\"";
            return false;
        }

        oPort.open("/test-superquadric:o");
        iPort.open("/test-superquadric:i");

        if (streaming_mode)
        {

            if (!Network::connect(oPort.getName(),"/superquadric-model/point:i") ||
                !Network::connect("/superquadric-model/superq:o",iPort.getName()))
            {
                yError()<<"Unable to connect to superquadric-model";
                close();
                return false;
            }
        }
        else
        {
            superqRpc.open("/test-superquadric/rpc:i");
            if (!Network::connect(superqRpc.getName(),"/superquadric-model/rpc"))
            {
                yError()<<"Unable to connect to superquadric-model rpc ";
                close();
                return false;
            }
        }

        if (rf.check("remove-outliers"))
            if (const Bottle *ptr=rf.find("remove-outliers").asList())
                outliersRemovalOptions=*ptr;

        sample=(unsigned int)rf.check("sample",Value(50)).asInt();
        inside_penalty=rf.check("inside-penalty",Value(100.0)).asDouble();
        test_derivative=rf.check("test-derivative");

        removeOutliers();
        sampleInliers();

        vtk_all_points=unique_ptr<Points>(new Points(all_points,2));
        vtk_out_points=unique_ptr<Points>(new Points(out_points,4));
        vtk_dwn_points=unique_ptr<Points>(new Points(dwn_points,1));

        vtk_all_points->set_colors(all_colors);
        vtk_out_points->get_actor()->GetProperty()->SetColor(1.0,0.0,0.0);
        vtk_dwn_points->get_actor()->GetProperty()->SetColor(1.0,1.0,0.0);

        Bottle &oBottle=oPort.prepare();
        oBottle.clear();
        Bottle &payLoad=oBottle.addList();



        Vector r_finitediff;
        if (streaming_mode)
        {
            for (auto &p:dwn_points)
            {
                Bottle &b=payLoad.addList();
                b.read(p);
            }
            oPort.writeStrict();

            while (true)
            {
                Property *iProp=iPort.read();
                yInfo()<<"received property:"<<iProp->toString();

                Vector v;
                r_finitediff.clear();
                iProp->find("center").asList()->write(v); r_finitediff=cat(r_finitediff,v);
                iProp->find("orientation").asList()->write(v); r_finitediff=cat(r_finitediff,v);
                iProp->find("dimensions").asList()->write(v); r_finitediff=cat(r_finitediff,v);
                iProp->find("exponents").asList()->write(v); r_finitediff=cat(r_finitediff,v);

                if (norm(r_finitediff.subVector(7,9))>0.0)
                    break;
            }
        }
        else
        {
            Bottle cmd, superq_b;
            cmd.addString("send_point_clouds");

            Bottle &in1=cmd.addList();

            for (size_t i=0; i<dwn_points.size(); i++)
            {
                Bottle &in=in1.addList();
                in.addDouble(dwn_points[i][0]);
                in.addDouble(dwn_points[i][1]);
                in.addDouble(dwn_points[i][2]);
                in.addDouble(dwn_points[i][3]);
                in.addDouble(dwn_points[i][4]);
                in.addDouble(dwn_points[i][5]);
            }

            superqRpc.write(cmd, superq_b);

            cmd.clear();
            cmd.addString("get_superq");
            superqRpc.write(cmd, superq_b);

            yInfo()<<"Received superquadric: "<<superq_b.toString();


            Vector v;
            v = getBottle(superq_b);
            r_finitediff.resize(12,0.0);
            Vector orient = dcm2euler(axis2dcm(v.subVector(8,11)));

            //yInfo()<<"Received superquadric: "<<orient.toString();
            r_finitediff.setSubvector(0, v.subVector(5,7));
            r_finitediff.setSubvector(3, v.subVector(8,11));
            r_finitediff.setSubvector(7, v.subVector(0,2));
            r_finitediff.setSubvector(10, v.subVector(3,4));
            r_finitediff[6] = - r_finitediff[6];

            yInfo()<<"Read superquadric: "<<r_finitediff.toString();

        }

        yInfo()<<"final superquadric_finitediff parameters:"<<r_finitediff.toString(3,3);
        unique_ptr<Superquadric> vtk_superquadric_finitediff=unique_ptr<Superquadric>(new Superquadric(r_finitediff,2.0));

        Vector r_analytic=findSuperquadric();
        unique_ptr<Superquadric> vtk_superquadric_analytic=unique_ptr<Superquadric>(new Superquadric(r_analytic,1.2));

        vtkSmartPointer<vtkRenderer> vtk_renderer=vtkSmartPointer<vtkRenderer>::New();
        vtkSmartPointer<vtkRenderWindow> vtk_renderWindow=vtkSmartPointer<vtkRenderWindow>::New();
        vtk_renderWindow->SetSize(300,300);
        vtk_renderWindow->AddRenderer(vtk_renderer);
        vtkSmartPointer<vtkRenderWindowInteractor> vtk_renderWindowInteractor=vtkSmartPointer<vtkRenderWindowInteractor>::New();
        vtk_renderWindowInteractor->SetRenderWindow(vtk_renderWindow);

        vtk_renderer->AddActor(vtk_all_points->get_actor());
        vtk_renderer->AddActor(vtk_out_points->get_actor());
        if (dwn_points.size()!=in_points.size())
            vtk_renderer->AddActor(vtk_dwn_points->get_actor());
        vtk_renderer->AddActor(vtk_superquadric_analytic->get_actor());
        vtk_renderer->AddActor(vtk_superquadric_finitediff->get_actor());
        vtk_renderer->SetBackground(0.1,0.2,0.2);

        vtkSmartPointer<vtkAxesActor> vtk_axes=vtkSmartPointer<vtkAxesActor>::New();     
        vtkSmartPointer<vtkOrientationMarkerWidget> vtk_widget=vtkSmartPointer<vtkOrientationMarkerWidget>::New();
        vtk_widget->SetOutlineColor(0.9300,0.5700,0.1300);
        vtk_widget->SetOrientationMarker(vtk_axes);
        vtk_widget->SetInteractor(vtk_renderWindowInteractor);
        vtk_widget->SetViewport(0.0,0.0,0.2,0.2);
        vtk_widget->SetEnabled(1);
        vtk_widget->InteractiveOn();

        vector<double> bounds(6),centroid(3);
        vtk_all_points->get_polydata()->GetBounds(bounds.data());
        for (size_t i=0; i<centroid.size(); i++)
            centroid[i]=0.5*(bounds[i<<1]+bounds[(i<<1)+1]);

        vtkSmartPointer<vtkCamera> vtk_camera=vtkSmartPointer<vtkCamera>::New();
        vtk_camera->SetPosition(centroid[0]+1.0,centroid[1],centroid[2]+0.5);
        vtk_camera->SetFocalPoint(centroid.data());
        vtk_camera->SetViewUp(0.0,0.0,1.0);
        vtk_renderer->SetActiveCamera(vtk_camera);

        vtkSmartPointer<vtkInteractorStyleSwitch> vtk_style=vtkSmartPointer<vtkInteractorStyleSwitch>::New();
        vtk_style->SetCurrentStyleToTrackballCamera();
        vtk_renderWindowInteractor->SetInteractorStyle(vtk_style);
        vtk_renderWindowInteractor->Start();
        
        return true;
    }

    /****************************************************************/
    bool updateModule() override
    {
        return false;
    }

    /****************************************************************/
    bool close() override
    {
        oPort.close();
        iPort.close();
        return true;
    }
};


/****************************************************************/
int main(int argc, char *argv[])
{
    Network yarp;
    if (!yarp.checkNetwork())
    {
        yError()<<"Unable to find Yarp server!";
        return EXIT_FAILURE;
    }

    ResourceFinder rf;
    rf.configure(argc,argv);

    Finder finder;
    return finder.runModule(rf);
}

