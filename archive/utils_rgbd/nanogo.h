#ifndef NanoG2O_define
#define NanoG2O_define
#include "Eigen/Core"
#include "Eigen/Sparse"
#include <vector>
#include <set>
#include <map>
#include <chrono>
#include <iomanip>
#include <iostream>
namespace nanogo{
//#define nanogo_DevDebug

/**A variable that needs optimization
 */
template<typename Type>
class Variable :public Eigen::Matrix<Type, Eigen::Dynamic, 1>{
public:
    bool fixed=false;//indicates whether this variable must be optimized or not
    bool needPreparation=false;//indicates whether this variable needs preparation before the call to compute to do some stuff
    virtual inline void prepare(){};//function to prepare before computing errors

    //epsilon value employed to automatically find the partial derivative of this variable
    Type derv_epsilon=1e-4;
    bool marginalize=false;
private:
    template<class U>    friend class SparseGraphSolver;
    template<class U>    friend class Graph;
    int64_t _JtJIdx=-1;//index of row where this variable starts in the JtJ Matrix

};

/**Error to be minimized
 */
template<typename Type >
class Error:public std::vector<Variable<Type> *>{
public:
    //must reimplement this function to compute the error
    virtual void compute(Eigen::Matrix<Type, Eigen::Dynamic, 1> &error)=0;
    //Re-Implement this function to compute the partial derivatives of the error wrt each variable
    //@param var of this Error for which the partial derivative must be computed
    //@param dervMatrix  N(error)x N(var) matrix with partial derivatives.
    //                 Number of rows is N(error) size of this error vector (computed in compute())
    //                 Number of cols is N(var), size of the Variable vector
    virtual inline void derv(uint32_t var,Eigen::Matrix<Type, Eigen::Dynamic,Eigen::Dynamic> &dervMatrix ){}

    //sets the information matrix. Assuming only diagonal elements, all with the same value
    inline void setInformation(double information){_sqr_information=sqrt(information);}
    //set hubber  loss as a robust estimator and delta as the parameter value for it
    inline void setDeltaHubber(double delta){_deltaHubber=delta; _deltaHubber2=delta*delta;}
    //Returns the Chi2 of this
    inline Type chi2(){return Chi2;}
private:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    template<class U>    friend class SparseGraphSolver;
    inline void compute_();//computes the error by calling first to Variable::prepare and then Error::compute
    inline void derv_();//computes partial derivatives, either calling  Eigen::derv  or Eigen::autoderive
    inline void autoderive_(Variable<Type> &var, Eigen::Matrix<Type, Eigen::Dynamic,Eigen::Dynamic> &dervMatrix );
    inline Type hubber(Type e,Type _delta);
    Type _sqr_information=1;
    Eigen::Matrix<Type, Eigen::Dynamic, 1> _error;//vector with the last call to error
    std::vector<Eigen::Matrix<Type, Eigen::Dynamic,Eigen::Dynamic>> _derv;//set of partial derivatives computed
    std::vector<bool> needsderv_;//for each var, indicates whether we must find the derivative or if is provided
    int64_t _ErrIdx=-1;//index of  where the error starts in the error vector
    Type _deltaHubber=0,_deltaHubber2=0,Chi2=0;

  };

/**A graph of errors
 */
template<typename Type>
class Graph{
  public:


    //
    inline void add(Error<Type> *err){
        _errors.push_back(err);
        for(size_t i=0;i<err->size();i++){//register for each var that it participates in the error
              if (err->at(i)->fixed )continue;
               var_errors[err->at(i)].insert({err,i});
               for(size_t j=0;j<err->size();j++)
                   if (i!=j){
                       if (!err->at(j)->fixed)
                           var_vars_error[err->at(i)][err->at(j)].push_back({err,i,j});
                   }
        }
        isPrepared=false;
    }

    void prepare(){
        if(isPrepared) return;
        //add the variables of the errors
        for(auto &err:_errors){
            for(auto &var:*err){
                if(var->_JtJIdx==-1)
                    add(var);
            }
        }
        isPrepared=true;
    }
    inline void add(Variable<Type> *v){
        if(!v->fixed){
            v->_JtJIdx=curVarIdx;
            curVarIdx+=v->size();
        }
         _vars.push_back(v);
    }

private:
    template<class U>    friend class SparseGraphSolver;
    std::vector<Variable<Type>*> _vars;
    std::vector<Error<Type>*> _errors;
    uint64_t curVarIdx=0;
    //for each var, the errors in which it participates. Used for computation of Jt*J
    std::map<Variable<Type> *,std::set<std::pair<Error<Type> *,int> > > var_errors;
    // Graph of variable coocurrencies. Each element is the set of errors in which a pair of var interact
    struct Einfo{
         Error<Type> *err;
         size_t first,second;
         inline bool operator<(const Einfo&e)const{return err<e.err;}
    };
    std::map<Variable<Type> *, std::map<  Variable<Type> *, std::vector< Einfo> >  >  var_vars_error;

    bool isPrepared=false;


};

/**Solves the graph using sparse matrices
 */
template<typename Type>
class SparseGraphSolver{
public:
    struct Params{
        Params(){}
        Params(int _maxIters,Type _minError,Type _min_step_error_diff=0,Type _tau=1  ){
            maxIters=_maxIters;
            minError=_minError;
            min_step_error_diff=_min_step_error_diff;
            tau=_tau;
         }

        int maxIters=100;//maximum number of iterations
        Type minError=1e-5; //minimum error. Below this optimization stops
        Type min_step_error_diff=0; //if error reduction between iterations is below this, optimization stops
        Type tau=1 ; //indicating how near the initial solution is estimated to be to the real one. If 1, it means that it is very far and the first
        bool verbose=false;
     };


    void solve(Graph<Type>&g,Params params=Params());

private:
    bool step_(Graph<Type>&g);
     void initialize_(Graph<Type>&g);
    void add_missing_diagonal_elements(Eigen::SparseMatrix<Type> &M);
    void get_diagonal_elements_refs_and_add( Eigen::SparseMatrix<Type> &M,std::vector<Type*> &refs,Type add_val);
    Type getChi2(Graph<Type>&g);


    void computeJtJ(Graph<Type>&g);
    Type currChi2,prevChi2,dumpFactor=-1,df_increase;
    Params _params;

    uint32_t  _varSize=0,_errorSize=0;
    Eigen::SparseMatrix<Type>  _J;
    Eigen::Matrix<Type, Eigen::Dynamic, 1> _Error;

    std::vector<Type*> _Jrefs;//,_JtJrefs,_JDiag,_JtJ;
  //  std::vector<uint32_t> JtJColumnSize;
    std::vector<uint32_t> JColumnSize;
//    Eigen::ConjugateGradient<Eigen::SparseMatrix<Type>, Eigen::Upper> solver;

};

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////   Class Error
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


template<typename Type>
Type Error<Type>::hubber(Type e,Type _delta){
    Type dsqr = _delta * _delta;
    if (e <= dsqr) { // inlier
        return  e;
    } else { // outlier
        Type sqrte = sqrt(e); // absolut value of the error
        return  2*sqrte*_delta - dsqr; // rho(e)   = 2 * delta * e^(1/2) - delta^2
    }
}
/////////////////
/////////////////
template<typename Type>
void  Error<Type>::compute_(){
//    for(auto &var:*this)
//        if (var->needPreparation) var->prepare();
    compute(_error);
    _error*=_sqr_information;
    if (_deltaHubber>0){
        //now, use the information and hubber loss if required
        Chi2=_error.cwiseProduct(_error).sum();
        if (Chi2>_deltaHubber2)
            _error*=sqrt( (2*sqrt(Chi2)*_deltaHubber - _deltaHubber2 )/ Chi2); // rho(e)   = 2 * delta * e^(1/2) - delta^2
    }
    Chi2=_error.cwiseProduct(_error).sum();
}

/////////////////
/////////////////
template<typename Type>
void  Error<Type>::derv_(){
    assert(_error.size()!=0);

    if (_derv.size()==0){//first time
        _derv.resize(this->size());
        for(size_t i=0;i<Error<Type>::size();i++){
            auto &var=*Error<Type>::at(i);
            if (!var.fixed){
                //call and see if does anything
                derv(i,_derv[i]);
                if (_derv[i].cols()==0){//the function did nothing. So,autoderive
                    needsderv_.push_back(true);
                    autoderive_(var,_derv[i]);
                }
                else{//partial derivatives provided
                    //check correct size
                    assert(_derv[i].rows()==_error.size() &&"Partial derivatives rows!=Error size");
                    assert(_derv[i].cols()==int(var.size())&&"Partial derivatives cols!=Size of the variable");
                    needsderv_.push_back(false);
                }
            }
            else{
                needsderv_.push_back(false);
            }

        }
    }
    else{//not first time, we already know who must be autoderived
        for(size_t i=0;i<Error<Type>::size();i++){
            auto &var=*Error<Type>::at(i);
            if (!var.fixed){
                if (needsderv_[i]) autoderive_(var,_derv[i]);
                else derv(i,_derv[i]);
            }
        }
    }

}


/////////////////
/////////////////
template<typename Type>
void Error<Type>::autoderive_(Variable<Type> &var,Eigen::Matrix<Type, Eigen::Dynamic,Eigen::Dynamic> &dervMatrix   ){
    assert(!var.fixed);
    assert(_error.size()!=0);
    Eigen::Matrix<Type, Eigen::Dynamic, 1> _error_plus,_error_minus;
    Type invEpsilon=1./(2*var.derv_epsilon);
    dervMatrix.resize(_error.size(),var.size());
    //for each dimension
    for(int d=0;d<var.size();d++ ){
        Type curVal=var(d);
        var(d)=curVal+var.derv_epsilon;
        if (var.needPreparation) var.prepare();
        compute(_error_plus);
        var(d)=curVal-var.derv_epsilon;
        if (var.needPreparation) var.prepare();
        compute(_error_minus);
        //partial derivative
        dervMatrix.col(d)=((_error_plus-_error_minus)*invEpsilon);
        //Set the initial value
        var(d)=curVal;
    }
    //a final call to prepare to leave thnigs as they were
//    if (var.needPreparation) var.prepare();

}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////   Class SparseGraphSolver
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

template<typename Type>
Type SparseGraphSolver<Type>:: getChi2(Graph<Type>&g){
    Type Chi2=0;
    for(auto e:g._errors) Chi2+=e->chi2();
    return Chi2;
}/*
#ifndef splm_get_time
#define splm_get_time(a,b) std::chrono::duration_cast<std::chrono::duration<Type>>(a-b).count()
#endif*/
/////////////////
/////////////////
template<typename Type>
void SparseGraphSolver<Type>:: solve(Graph<Type>&g,Params  params){

    _params=params;
    g.prepare();
    initialize_(g);
    prevChi2=getChi2(g);
    if (_params.verbose) std::cout<<"Initial Chi2="<<prevChi2<<std::endl;
    //intial error estimation
    int mustExit=0;
    for ( int i = 0; i < _params.maxIters && !mustExit; i++ ) {
        auto start= std::chrono::high_resolution_clock::now();
        if (_params.verbose)std::cerr<<"iteration "<<i<<"/"<<_params.maxIters<< "  ";
        bool isStepAccepted=step_(g );
        //check if we must exit
        if ( currChi2<_params.minError ) mustExit=1;
        if( fabs( prevChi2 -currChi2)<=_params.min_step_error_diff  || !isStepAccepted) mustExit=2;
        //exit if error increment
        if (currChi2>prevChi2  )mustExit=3;
        //  if (_step_callback) _step_callback(_E);
        prevChi2=currChi2;
        if (_params.verbose) std::cout<<"Iteration "<<i<<" Chi2="<<currChi2<<std::endl;
        auto now= std::chrono::high_resolution_clock::now();
        //std::cout<<"time it("<<i<<")"<< splm_get_time(now,start)<<std::endl;

    }
    //    std::cout<<"Exit code="<<mustExit<<std::endl;
    //return currErr;
}


template<typename Type>
void SparseGraphSolver<Type>:: initialize_(Graph<Type>&graph){
    //compute the number of variables if not known yet (first time)
    _varSize=0;
    dumpFactor=-1;

    for(auto &var:graph._vars){
        if (!var->fixed)  _varSize+=var->size();
        assert(var->_JtJIdx>=0);
    }
    _errorSize=0;
    //first, compute the errors
    for(auto &var:graph._vars) //prepare variables that needs to
        if(var->needPreparation) var->prepare();
    for(auto &error:graph._errors){
        //first compute error
        error->compute_();
        error->_ErrIdx=_errorSize;
        _errorSize+=error->_error.size();
    }
    _Error.resize(_errorSize);
    //fill the error vector with the errors
    for(auto &error:graph._errors)
     _Error.middleRows(error->_ErrIdx,error->_error.size())=error->_error;


    ///////////////////////////////////
    //Prepare J
    //compute how many elements are in each column of J
     JColumnSize=std::vector<uint32_t>(_varSize,0);
    uint32_t JtotalSize=0;
    for(auto &error:graph._errors){
        int errsize=error->_error.size();
         for(auto var: *error)
            if (!var->fixed) {
                for(int i=0;i<var->size();i++){
                    JColumnSize[var->_JtJIdx+i]+=errsize;
                    JtotalSize+=errsize;
                }
            }
    }
    _J.resize(_errorSize,_varSize );
    _J.reserve(JColumnSize);



    //insert the elements as zeros
    _Jrefs.clear();
    _Jrefs.reserve(JtotalSize);
    for(auto &error:graph._errors){
        for(auto var: *error){
            if (var->fixed) continue;
            for(int r=0;r<error->_error.size();r++)
                for(int c=0;c<var->size();c++){
                    _Jrefs.push_back(&_J.insert(error->_ErrIdx+r,var->_JtJIdx+c  ));
                    *(_Jrefs.back())=0;
                }
        }
    }

    _J.makeCompressed();

}


template<typename T>
void SparseGraphSolver<T>::get_diagonal_elements_refs_and_add( Eigen::SparseMatrix<T> &M,std::vector<T*> &refs,T add_val){

    refs.resize(M.cols());
    //now, get their references and add mu
    for (int k=0; k<M.outerSize(); ++k)
        for ( typename Eigen::SparseMatrix<T>::InnerIterator it(M,k); it; ++it)
            if (it.row()== it.col())     {refs[it.row()]= &it.valueRef(); *refs[it.row()]+=add_val;}
}


template<typename T>
void SparseGraphSolver<T>::add_missing_diagonal_elements(Eigen::SparseMatrix<T> &M) {
  std::vector<bool> diag(M.rows(),false);
  for (int k=0; k<M.outerSize(); ++k)
     for (  typename Eigen::SparseMatrix<T>::InnerIterator it(M,k); it; ++it)
          if (it.row()== it.col())     diag[it.row()]=true;
  //and add them
  for(size_t i=0;i<diag.size();i++)   if (!diag[i]) M.insert(i,i) =0;

}





template<typename Type>
bool SparseGraphSolver<Type>:: step_(Graph<Type>&graph){

    typedef  Eigen::Matrix<Type, Eigen::Dynamic, 1>  eVector;
#ifdef nanogo_DevDebug
    std::vector<Eigen::Triplet<Type> > JTriplets; //develop debug
#endif
    auto t1= std::chrono::high_resolution_clock::now();


    //compute partial derivatives and Jt
       size_t jrefidx=0;
     for(auto &error:graph._errors){
        error->derv_();
        for(size_t v=0;v<error->size();v++)
        {
            Variable<Type> &var=(*error->at(v));
            if (!var.fixed){
                //move the partial derivatives here
                auto &derv_mat=error->_derv[v];
                for(int r=0;r<derv_mat.rows();r++)
                    for(int c=0;c<derv_mat.cols();c++){
                        *_Jrefs[jrefidx++]=derv_mat(r,c);
                    }
         }
        }
    }
     auto t2= std::chrono::high_resolution_clock::now();


#ifdef nanogo_DevDebug
    Eigen::SparseMatrix<double> auxMatrix(_errorSize,_varSize);
    auxMatrix.setFromTriplets(JTriplets.begin(),JTriplets.end());
     assert(  fabs((auxMatrix-_J).norm())<1e-3) ;
#endif
     Eigen::SparseMatrix<Type> JtJ=_J.transpose()*_J;


    auto t3= std::chrono::high_resolution_clock::now();

    Eigen::Matrix<Type, Eigen::Dynamic, 1>  B=-_J.transpose()*_Error;
    auto t4= std::chrono::high_resolution_clock::now();

    if(dumpFactor<0){//first time only
        Type maxv=std::numeric_limits<Type>::lowest();
        for (int k=0; k<JtJ.outerSize(); ++k)
            for (typename Eigen::SparseMatrix<Type>::InnerIterator it(JtJ,k); it; ++it)
                if (it.row()== it.col())
                    if (it.value()>maxv)
                        maxv=it.value();
        dumpFactor=maxv*_params.tau;
    }

    Type gain=0,prev_dumpFactor=0;

    int ntries=0;
    bool isStepAccepted=false;
    auto t5= std::chrono::high_resolution_clock::now();

    std::vector<Type*> refs;
    do{
        //add dumping factor to JtJ.
#if 1 //very efficient in any case, but particularly if initial dump does not produce improvement and must reenter
        auto _t1= std::chrono::high_resolution_clock::now();


        if(refs.size()==0){//first time into the do
            add_missing_diagonal_elements(JtJ);
            get_diagonal_elements_refs_and_add(JtJ,refs,dumpFactor);
        }
        else for(size_t i=0;i<refs.size();i++)    *refs[i]+= dumpFactor-prev_dumpFactor;//update mu


        prev_dumpFactor=dumpFactor;
        auto _t2= std::chrono::high_resolution_clock::now();

#else  //less efficient, but easier to understand
        Eigen::SparseMatrix<T> A=JtJ+I*dumpFactor;
        Eigen::SimplicialLDLT<Eigen::SparseMatrix<T> > chol(A);  // performs a Cholesky
#endif

         Eigen::SimplicialLDLT<Eigen::SparseMatrix<Type> > chol(JtJ);  // performs a Cholesky
        eVector  delta= chol.solve(B);

        //Eigen::ConjugateGradient<Eigen::SparseMatrix<double>, Eigen::Upper> solver;
        auto _t3= std::chrono::high_resolution_clock::now();
        //apply delta
        for(auto var:graph._vars)
            (*var)+=delta.middleRows(var->_JtJIdx,var->size());
        auto _t4= std::chrono::high_resolution_clock::now();


        //re-compute errors while filling the error vector
        Type cChi2=0;

        for(auto &var:graph._vars) //prepare variables that needs to
            if(var->needPreparation) var->prepare();
        for(auto &error:graph._errors){
            error->compute_();
             _Error.middleRows(error->_ErrIdx,error->_error.size())=error->_error;
            cChi2+=error->chi2();
        }

        auto _t5= std::chrono::high_resolution_clock::now();

        auto L=0.5*delta.transpose()*((dumpFactor*delta) - B);
        gain= (cChi2-prevChi2)/ L(0,0) ;
        //get gain
        if (gain>0 && ((cChi2-prevChi2)<0)){
            dumpFactor=dumpFactor*std::max(Type(0.33),Type(1.-pow(2*gain-1,3)));
            df_increase=2.f;
            currChi2=cChi2;
            isStepAccepted=true;
        }
        else{

            dumpFactor=dumpFactor*df_increase;
            df_increase=df_increase*5;
            //restore previous solution
            for(auto var:graph._vars)
                (*var)-=delta.middleRows(var->_JtJIdx,var->size());
        }
        auto _t6= std::chrono::high_resolution_clock::now();/*
        std::cout<<"--t12="<<splm_get_time(_t2,_t1)<<std::endl;
        std::cout<<"--t23="<<splm_get_time(_t3,_t2)<<std::endl;
        std::cout<<"--t34="<<splm_get_time(_t4,_t3)<<std::endl;
        std::cout<<"--t45="<<splm_get_time(_t5,_t4)<<std::endl;
        std::cout<<"--t56="<<splm_get_time(_t6,_t5)<<std::endl;
        std::cout<<"--t16="<<splm_get_time(_t6,_t1)<<std::endl;*/

    }while(  gain<=0 && ntries++<5 &&  !isStepAccepted);


    auto tend= std::chrono::high_resolution_clock::now();/*

    std::cout<<"t12(derv)="<<splm_get_time(t2,t1)<<std::endl;
    std::cout<<"t23(JtJ) ="<<splm_get_time(t3,t2)<<std::endl;
    std::cout<<"t34(B)   ="<<splm_get_time(t4,t3)<<std::endl;

    std::cout<<"t15="<<splm_get_time(t5,t1)<<std::endl;
    std::cout<<"t5end="<<splm_get_time(tend,t5)<<std::endl;
    std::cout<<"t1-end="<<splm_get_time(tend,t1)<<std::endl;*/
     if (_params.verbose) std::cout<<std::setprecision(5) <<"Curr Error="<<currChi2<<" AErr(prev-curr)="<<prevChi2-currChi2<<" gain="<<gain<<" dumping factor="<<dumpFactor<<std::endl;



    //  if (_params.verbose) {std::cerr<<" J="<<splm_get_time(t2,t1)<<" transpose="<< splm_get_time(t22,t2)<<" Jt*J="<< splm_get_time(t3,t22)<<" B="<< splm_get_time(t4,t3) <<" chol="<< splm_get_time(t6,t5) <<std::endl;
    // std::cerr<<"solve="<<T(t4-t3)/T(CLOCKS_PER_SEC)<<std::endl;
    //}
    return isStepAccepted;
}

}

#endif
