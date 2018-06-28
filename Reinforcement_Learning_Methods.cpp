// Implemented by Aslan Mehrabi

// Implementation of Reinforcement Learning Methods
// Implemented Algorithms: 
// + e-soft on-policy Monte Carlo control
// + First-Visit MC
// + Temporal difference [TD(0)]
// + Actor-critic 
// + Gibbs softmax


#include <iostream>
#include <algorithm>
#include <string>
#include <math.h>

using namespace std;

#define LEN 6
#define EPS 0.1
#define ALPHA 0.1
#define GAMA 0.9
#define E 2.7182815
#define GAMA22 0.8
#define ALPHA22 0.1
#define BETA22 0.01

enum {UP,DOWN,RIGHT,LEFT,FINISH};


int moveX[]={0,0,1,-1};
int moveY[]={1,-1,0,0};



double poison[LEN][LEN][4][10]; //4: directions, 10: values of poison
double probPoison[LEN][LEN][4][10];
int numPoison[LEN][LEN][4];  
double pi[LEN][LEN][4]; //if not valid =>-1
double q[LEN][LEN][4];
int numValidMove[LEN][LEN];


int lenX,lenY,goalX,goalY,stX,stY;

int policy[LEN][LEN];


double  q11numRun,q11numIter,q12numRun,q12numIter;
double  q21numRun,q21numIter,q22numRun,q22tresh;
bool readMap(){
	char stmp[1000];
	char name[1000];
	float tmp1,tmp2;
	
	cin>>stmp>>q11numIter>>q11numRun>>stmp>>q12numIter>>q12numRun;
	cin>>stmp>>q21numIter>>q21numRun>>stmp>>q22tresh>>q22numRun;
	int tmp,tmpNum,numFault;
	cin>>lenY>>lenX>>tmpNum>>numFault;	
	cin>>tmp;
	tmp--;
	stY=tmp/lenX;
	stX=tmp%lenX;

	cin>>tmp;
	tmp--;
	goalY=tmp/lenX;
	goalX=tmp%lenX;

	for(int i=0;i<numFault;i++){
		int tmp1,tmp2;
		int x1,x2,y1,y2;
		cin>>tmp1>>tmp2;
		tmp1--;
		tmp2--;
		y1=tmp1/lenX;
		x1=tmp1%lenX;
		y2=tmp2/lenX;
		x2=tmp2%lenX;

		if(y2>y1 || x2>x1){//swap
			y1=y2+y1;
			y2=y1-y2;
			y1=y1-y2;

			x1=x1+x2;
			x2=x1-x2;
			x1=x1-x2;
		}
		cin>>tmp;//num of probabilities
		if(y1-y2==1){
			numPoison[y1][x1][DOWN]=tmp;
			numPoison[y2][x2][UP]=tmp;
			for(int i=0;i<tmp;i++){
				cin>>probPoison[y2][x2][UP][i]>>poison[y2][x2][UP][i];
				poison[y2][x2][UP][i]*=-1;
				poison[y1][x1][DOWN][i]=poison[y2][x2][UP][i];
				probPoison[y1][x1][DOWN][i]=probPoison[y2][x2][UP][i];
			}
		}
	
		else if(x1-x2==1){
			numPoison[y1][x1][LEFT]=tmp;
			numPoison[y2][x2][RIGHT]=tmp;
			for(int i=0;i<tmp;i++){
				cin>>probPoison[y2][x2][RIGHT][i]>>poison[y2][x2][RIGHT][i];
				poison[y2][x2][RIGHT][i]*=-1;
				poison[y1][x1][LEFT][i]=poison[y2][x2][RIGHT][i];
				probPoison[y1][x1][LEFT][i]=probPoison[y2][x2][RIGHT][i];
			}
		}
		else{
			cout<<"err: read1\n";
		}
	}
	

	int nextX,nextY;
	
	for(int i=0;i<lenY;i++){
		for(int j=0;j<lenX;j++){
			
			cin>>tmp;
			tmp--;
			nextY=tmp/lenX;
			nextX=tmp%lenX;

			if(nextY-i==1)
				policy[i][j]=UP;
			else if(nextY-i==-1)
				policy[i][j]=DOWN;
			else if(nextX-j==1)
				policy[i][j]=RIGHT;
			else if(nextX-j==-1)
				policy[i][j]=LEFT;
			else if(tmp==-2) //
				policy[i][j]=FINISH;
			else
				cout<<"err: read2\n";
		}
		
	}


	return true;
}



int PoisonRndValue(int y,int x,int direction){
	
	int rndI=rand();
	
	double rnd=(double)(rndI%1000)/1000;
	double sum=0;
	for(int i=0;i<numPoison[y][x][direction];i++){
		sum+=probPoison[y][x][direction][i];
		if(sum>=rnd)
			return poison[y][x][direction][i];
	}
}


double rew[LEN][LEN];
double totalRew[LEN][LEN];
double totalRewRun[LEN][LEN];
bool firstVisitMontCarlo(int numIterate,int numRun){

	for(int r=0;r<numRun;r++){
		//cout<<"1: "<<r<<endl;
		int sumRew=0;
		int x=stX,y=stY;
		memset(totalRew,0,sizeof totalRew);
		

		for(int i=0;i<numIterate;i++){
			x=stX;
			y=stY;
			sumRew=0;
			memset(rew,0,sizeof rew);

			while(x!=goalX || y!=goalY){
				int tmpRew=PoisonRndValue(y,x,policy[y][x]);
				sumRew+=tmpRew;
				if(policy[y][x]==UP){
					y=y+1;
					rew[y][x]+=rew[y-1][x];
					rew[y][x]+=tmpRew;
				}
				else if(policy[y][x]==DOWN){
					y=y-1;
					rew[y][x]+=rew[y+1][x];
					rew[y][x]+=tmpRew;
				}
				else if(policy[y][x]==RIGHT){
					x=x+1;
					rew[y][x]+=rew[y][x-1];
					rew[y][x]+=tmpRew;
				}
				else if(policy[y][x]==LEFT){
					x=x-1;
					rew[y][x]+=rew[y][x+1];
					rew[y][x]+=tmpRew;
				}
				else
					cout<<"err first\n";
			}
			if(x==goalX && y==goalY){
				//rew[y][x]+=1000;
				sumRew+=1000;
			}

			for(int i=0;i<lenY;i++)
				for(int j=0;j<lenX;j++){
					if(rew[i][j]==0 && (i!=stY || j!=stX ))
						continue;
					totalRew[i][j]+=sumRew-rew[i][j];
				}

		}

		for(int i=0;i<lenY;i++)
			for(int j=0;j<lenX;j++){
				totalRew[i][j]/=numIterate;
				totalRewRun[i][j]+=totalRew[i][j];
			}
	}

	for(int i=0;i<lenY;i++)
		for(int j=0;j<lenX;j++)
			totalRewRun[i][j]/=numRun;


	return true;
}

bool initialize1b(){
	for(int i=0;i<lenY;i++)
		for(int j=0;j<lenX;j++){
			numValidMove[i][j]=4;
			if(i==0 || i==lenY-1)
				numValidMove[i][j]--;
			if(j==0 || j==lenX-1)
				numValidMove[i][j]--;
			if(i!=0)
				pi[i][j][DOWN]=1.0/numValidMove[i][j];
			if(i!=lenY-1)
				pi[i][j][UP]=1.0/numValidMove[i][j];
			if(j!=0)
				pi[i][j][LEFT]=1.0/numValidMove[i][j];
			if(j!=lenX-1)
				pi[i][j][RIGHT]=1.0/numValidMove[i][j];
		}
		return true;
}

int move(int y,int x,double pi[LEN][LEN][4] ){
	
	int rndI=rand();
	
	double rnd=(double)(rndI%1000)/1000;
	double sum=0;
	for(int i=0;i<4;i++){
		if(pi[y][x][i]==0)
			continue;
		sum+=pi[y][x][i];
		if(rnd<=sum)
			return i;
	}

	cout<<"err move\n";
	return -1;
}


bool VisitedQ[LEN][LEN][4];
int numVisitQ[LEN][LEN][4];
double qTmp[LEN][LEN][4];
double qRun[LEN][LEN][4];
double piRun[LEN][LEN][4];
bool eSoftMontCarlo(int numIterate,int numRun){

	for(int r=0;r<numRun;r++){

		memset(q,0,sizeof q);
		memset(numVisitQ,0,sizeof numVisitQ);
		memset(qTmp,0,sizeof qTmp);
		initialize1b();
		
		


		int sumRew=0;
		int x=stX,y=stY;
		int lenCnt;
		for(int i=0;i<numIterate;i++){

			memset(VisitedQ,0,sizeof VisitedQ);
			x=stX;
			y=stY;
			sumRew=0;
			memset(rew,0,sizeof rew);
			lenCnt=0;
			while(x!=goalX || y!=goalY){
				lenCnt++;
				int moveDir=move(y,x,pi); //// @@@ test
				int tmpRew=PoisonRndValue(y,x,moveDir);

				numVisitQ[y][x][moveDir]++;
				if(!VisitedQ[y][x][moveDir]){
					qTmp[y][x][moveDir]=sumRew;
					VisitedQ[y][x][moveDir]=true;
				}
				sumRew+=tmpRew;


				if(moveDir==UP){
					y=y+1;
				}
				else if(moveDir==DOWN){
					y=y-1;
				}
				else if(moveDir==RIGHT){
					x=x+1;
				}
				else if(moveDir==LEFT){
					x=x-1;
				}
				else
					cout<<"err eSoft\n";
			}
			if(x==goalX && y==goalY){
				sumRew+=1000;
			}

			//change the policy
			for(int i=0;i<lenY;i++){
				for(int j=0;j<lenX;j++){
					int max=-1,maxInd;
					for(int d=0;d<4;d++){
						if(sumRew-qTmp[i][j][d]>max){
							if(VisitedQ[i][j][d]==false)
								continue;
							max=sumRew-qTmp[i][j][d];
							maxInd=d;
						}
					}
					if(max==-1)
						continue;
					if(pi[i][j][maxInd]==0)
						cout<<"errrr\n";
					pi[i][j][maxInd]=1-EPS+EPS/numValidMove[i][j];
					if(i!=lenY-1 && maxInd!=UP)
						pi[i][j][UP]=EPS/numValidMove[i][j];
					if(i!=0 && maxInd!=DOWN)
						pi[i][j][DOWN]=EPS/numValidMove[i][j];
					if(j!=lenX-1 && maxInd!=RIGHT)
						pi[i][j][RIGHT]=EPS/numValidMove[i][j];
					if(j!=0 && maxInd!=LEFT)
						pi[i][j][LEFT]=EPS/numValidMove[i][j];

				}
			}

			for(int i=0;i<lenY;i++)
				for(int j=0;j<lenX;j++)
					for(int d=0;d<4;d++){
						if(VisitedQ[i][j][d])
							q[i][j][d]+=sumRew-qTmp[i][j][d];
					}



		}

		for(int i=0;i<lenY;i++)
			for(int j=0;j<lenX;j++)
				for(int d=0;d<4;d++){
					if(numVisitQ[i][j][d]){
						q[i][j][d]/=numVisitQ[i][j][d];
						qRun[i][j][d]+=q[i][j][d];
						piRun[i][j][d]+=piRun[i][j][d];
					}
				}
	}
	for(int i=0;i<lenY;i++)
		for(int j=0;j<lenX;j++)
			for(int d=0;d<4;d++){
				qRun[i][j][d]/=numRun;
				piRun[i][j][d]/=numRun;
			}



	return true;
}


double v[LEN][LEN];
double vRun[LEN][LEN];
bool td0(int numIterate,int numRun){

	for(int r=0;r<numRun;r++){
		memset(v,0,sizeof v);
		int sumRew=0;
		int x=stX,y=stY;

		for(int i=0;i<numIterate;i++){
			x=stX;
			y=stY;
			sumRew=0;
			memset(rew,0,sizeof rew);

			while(x!=goalX || y!=goalY){
				int tmpRew=PoisonRndValue(y,x,policy[y][x]);
				sumRew+=tmpRew;
				if(policy[y][x]==UP){
					if(goalX==x && goalY==y+1)
						tmpRew+=1000;
					v[y][x] += ALPHA*(tmpRew+GAMA*v[y+1][x]-v[y][x]);
					y=y+1;
				}

				else if(policy[y][x]==DOWN){
					if(goalX==x && goalY==y-1)
						tmpRew+=1000;
					v[y][x] += ALPHA*(tmpRew+GAMA*v[y-1][x]-v[y][x]);
					y=y-1;
				}
				else if(policy[y][x]==RIGHT){
					if(goalX==x+1 && goalY==y)
						tmpRew+=1000;
					v[y][x] += ALPHA*(tmpRew+GAMA*v[y][x+1]-v[y][x]);
					x=x+1;
				}
				else if(policy[y][x]==LEFT){
					if(goalX==x-1 && goalY==y)
						tmpRew+=1000;
					v[y][x] += ALPHA*(tmpRew+GAMA*v[y][x-1]-v[y][x]);
					x=x-1;
				}
				else
					cout<<"err first\n";
			}
			
		}
		for(int i=0;i<lenY;i++)
			for(int j=0;j<lenX;j++)
				vRun[i][j]+=v[i][j];
	}

	for(int i=0;i<lenY;i++)
		for(int j=0;j<lenX;j++){
			vRun[i][j]/=numRun;
		}


	return true;
}


double pay [LEN][LEN][4];//3.14 unani
double v2[LEN][LEN];
//  pi=> hamun p


void calcPay(){
	for(int i=0;i<lenY;i++){
		for(int j=0;j<lenX;j++){
			double sum=0;
			for(int d=0;d<4;d++){
				if((i==lenY-1 && d==UP) || (i==0 && d==DOWN) || (j==lenX-1 && d==RIGHT) || (j==0 && d==LEFT)){
					continue;
				}
				sum+=pow((double)E,(double)pi[i][j][d]);
			}
			for(int d=0;d<4;d++){
				if((i==lenY-1 && d==UP) || (i==0 && d==DOWN) || (j==lenX-1 && d==RIGHT) || (j==0 && d==LEFT)){
					continue;
				}
				pay[i][j][d]=pow((double)E,(double)pi[i][j][d])/sum;
			}
		}

	}
}

void calcPayPoint(int y,int x){
	int i=y;
	int j=x;
	double sum=0;
	for(int d=0;d<4;d++){
		if((i==lenY-1 && d==UP) || (i==0 && d==DOWN) || (j==lenX-1 && d==RIGHT) || (j==0 && d==LEFT)){
			continue;
		}
		sum+=pow((double)E,(double)pi[i][j][d]);
	}
	for(int d=0;d<4;d++){
		if((i==lenY-1 && d==UP) || (i==0 && d==DOWN) || (j==lenX-1 && d==RIGHT) || (j==0 && d==LEFT)){
			continue;
		}
		pay[i][j][d]=pow((double)E,(double)pi[i][j][d])/sum;
	}
}

bool visitedMoveGreedy[LEN][LEN];
double moveGreedyPolicy(int y,int x,double p[LEN][LEN][4],bool print){

	
	memset(visitedMoveGreedy,0,sizeof visitedMoveGreedy);
	double ans=1;
	while(x!=goalX || y!=goalY){
		if(print)
			cout<<y<<' '<<x<<endl;
		if(visitedMoveGreedy[y][x]==true)
			return -1;
		visitedMoveGreedy[y][x]=true;
		double max=0;
		int maxInd=-1;
		for(int d=0;d<4;d++){
			if(p[y][x][d]>max){
				max=p[y][x][d];
				maxInd=d;
			}
		}
		if(maxInd==-1)
			cout<<"err moveGreedy\n";

		
		if(maxInd==UP){
			ans*=max;
			y=y+1;
			if(goalX==x && goalY==y)
				break;
		}

		else if(maxInd==DOWN){
			y=y-1;
			ans*=max;
			if(goalX==x && goalY==y)
				break;
		}
		else if(maxInd==RIGHT){
			ans*=max;
			x=x+1;
			if(goalX==x && goalY==y)
				break;
		}
		else if(maxInd==LEFT){
			ans*=max;
			x=x-1;
			if(goalX==x && goalY==y)
				break;
		}
		else
			cout<<"err move greedy2\n";
	}
	if(y==goalY && x==goalX && print)
		cout<<y<<' '<<x<<endl;
	return ans;
}


double payRun[LEN][LEN][4];
double v2Run[LEN][LEN];
bool actionValue(double threshold,int numRun){
	for(int r=0;r<numRun;r++){
		initialize1b();
		memset(v2,0,sizeof v2);
		double delta=0;

		int sumRew=0;
		int x=stX,y=stY;

		while(true){

			calcPay();

			memset(VisitedQ,0,sizeof VisitedQ);
			x=stX;
			y=stY;
			sumRew=0;

			while(x!=goalX || y!=goalY){
				if(y==4 && x==3)
					int aa=1;
				calcPayPoint(y,x);


				int moveDir=move(y,x,pay); //// @@@ test
				if(moveDir>3)
					int xx=0;
				int tmpRew=PoisonRndValue(y,x,moveDir);

				if(moveDir==UP){
					if(goalX==x && goalY==y+1)
						tmpRew+=1000;
					delta = tmpRew + GAMA22*v2[y+1][x]-v2[y][x];
					pi[y][x][moveDir]+=BETA22*delta;
					v2[y][x] += ALPHA*(delta);
					y=y+1;
				}

				else if(moveDir==DOWN){
					if(goalX==x && goalY==y-1)
						tmpRew+=1000;
					delta = tmpRew + GAMA22*v2[y-1][x]-v2[y][x];
					pi[y][x][moveDir]+=BETA22*delta;
					v2[y][x] += ALPHA*(delta);
					y=y-1;
				}
				else if(moveDir==RIGHT){
					if(goalX==x+1 && goalY==y)
						tmpRew+=1000;
					delta = tmpRew + GAMA22*v2[y][x+1]-v2[y][x];
					pi[y][x][moveDir]+=BETA22*delta;
					v2[y][x] += ALPHA*(delta);
					x=x+1;
				}
				else if(moveDir==LEFT){
					if(goalX==x-1 && goalY==y)
						tmpRew+=1000;
					delta = tmpRew + GAMA22*v2[y][x-1]-v2[y][x];
					pi[y][x][moveDir]+=BETA22*delta;
					v2[y][x] += ALPHA*(delta);
					x=x-1;
				}
				else
					cout<<"err action value\n";


			}//while
			
			double tmp=moveGreedyPolicy(stY,stX,pay,false);
			if(tmp>threshold){
				moveGreedyPolicy(stY,stX,pay,true);
				cout<<"policy accuray: "<<tmp<<endl;
				break;
			}

		}//while
		for(int i=0;i<lenY;i++)
			for(int j=0;j<lenX;j++){
				v2Run[i][j]+=v2[i][j];
				for(int d=0;d<4;d++)
					payRun[i][j][d]+=pay[i][j][d];
			}
	}

	for(int i=0;i<lenY;i++)
		for(int j=0;j<lenX;j++){
			v2Run[i][j]/=numRun;
			for(int d=0;d<4;d++){
				payRun[i][j][d]/=numRun;
			}
		}
		cout<<"=========\n";
		double tmp=moveGreedyPolicy(stY,stX,pay,true);
		cout<<"policy accuray: "<<tmp<<endl;

	return true;
}









int main(){
	


	freopen("input1.txt","r",stdin);
		
	readMap();

	cout<<1<<endl;
	int NumIterate=10000;
	firstVisitMontCarlo(q11numIter,q11numRun);
	//totalRew
	cout<<2<<endl;
	
	eSoftMontCarlo(q12numIter,q12numRun);
	//pi
	//q
	cout<<3<<endl;


	td0(q21numIter,q21numRun);  //khune akhar V 0 dare bejaye 1000
	//v
	cout<<4<<endl;
	actionValue(q22tresh,q22numRun);


	return 0;
}