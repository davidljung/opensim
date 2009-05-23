/****************************************************************************
  Copyright (C)2002 David Jung <opensim@pobox.com>

  This program/file is free software; you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation; either version 2 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details. (http://www.gnu.org)

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

  $Id: IKORFullSpaceSolver.cpp 1080 2004-07-28 19:51:26Z jungd $
  $Revision: 1.20 $
  $Date: 2004-07-28 15:51:26 -0400 (Wed, 28 Jul 2004) $
  $Author: jungd $

****************************************************************************/

#include <robot/control/kinematics/IKORFullSpaceSolver>

using robot::control::kinematics::IKORFullSpaceSolver;

//
// This file started as a 'transliteration' of the Full-Space-Parameterization
//  (FSP) solution code from the IKORv2.0 FSP.c file
//
//
// WARNING: This implementation isn't fully debugged and may not work
//  correctly.  Use FullSpaceSolver2.
//

using base::IVector;
using base::zeroIVector;
using base::equals;

const Real small = 5.0e-05; // (=def.SMALL)


IKORFullSpaceSolver::IKORFullSpaceSolver()
  : status(Unknown)
{
  Logln("Warning: This implementation doesn't work correctly, consider using FullSpaceSolver2 instead.");
}


// debugging convenience
#define dump_A() { \
                printf("A=\n"); \
                for(Int r=0; r<Nred; r++) { \
                  for(Int c=0; c<Mred; c++) { \
                    printf("%9.6f ",A(slRow[r], slCol[c])); \
                  } \
                  printf("\n"); \
                } \
                printf("\n"); \
              }

#define dump_Acol(c) { \
                printf("A[:,%d(%d)]=",(c),slCol[c]); \
                for(Int r=0; r<Nred; r++) { \
                  printf("%9.6f ",A(slRow[r], slCol[c])); \
                } \
                printf("\n"); \
              }

#define dump_btemp() { \
                       printf("btemp=["); \
                       for(Int r=0; r<Nred; r++) { \
                         printf("%9.6f",btemp[slRow[r]]); \
                         if (r!=Nred-1) printf(","); \
                       } \
                       printf("]\n"); \
                     }


base::Matrix IKORFullSpaceSolver::solve(const Matrix& A_in, const Vector& b_in,
                                        array<Int>& dependentRowsEliminated)
{
  // Note:
  //  Variables that correspond to variables from the original IKORv2.0
  //  code are marked with (=<correspinding-name>); (=id) if the
  //  names are identical.

  //
  // Initialization/Setup
  //

  // make a local copy so we can modify
  Matrix A(A_in); // (=Aorig)
  Vector b(b_in); // (=borig)
Debugln(Tmp,"\nSOLVE: A=\n" << A << "b=" << b);
  const Int N = A.size1(); // (=global N)
  const Int M = A.size2(); // (=global M)

  Assert(b.size()==N);

  Matrix Ared(N,M); // reduced A (=id)
  Vector bred(N);   // reduced b (=id)

  // array that corresponds to the values in the first soln:
  //  !0 ==> non-zero value, 0 ==> zero value
  IVector firstOK(N); // (=id)
  firstOK = zeroIVector(N);

  IVector rowElim(N); // indicates if a row of A has been eliminated (and why - see consts) (=RowElim)
  IVector colElim(M); // indicated if a column of A has been eliminated (and why - see consts) (=ColElim)
  rowElim = zeroIVector(N); // init to NotEleminated==0
  colElim = zeroIVector(M);

  systemComplete = false;

  //
  // Solve
  //

  bool bIsZero = equals(b,zeroVector(N),small); // is b completely zero? (=bcheck)



  //
  // Reduce A
  Int Nred = N; // reduced N (=FSP_data.Nred)
  Int Mred = M; // reduced M (=FSP_data.Mred)
  Matrix specialg(M-1,M); // null vectors created by specialCase1 (=Specialg)
  Int numSpecialgs; // number of g vectors created by specialCase1 (=NumSpg)

  // Eliminate all the nonredundancies from the A and B
  reduceA(A, Ared, b, bred, M, N, Mred, Nred, colElim, rowElim, specialg, numSpecialgs);

  Matrix g(Mred-Nred+1+numSpecialgs, M); // array of gi solution vectors (=id)
  g = zeroMatrix(g.size1(), g.size2()); // init to all 0s

  // Debugging output
  Debugln(FSP,"\n\n-------------- FSP ---------------");
  Debugcln(FSP," Ared:\n" << Ared);
  Debugcln(FSP," bred: " << bred);
  Debugcln(FSP,"M:" << M << " Mred:" << Mred << " N:" << N << " Nred:" << Nred);
  Debugcln(FSP,"rowElim:" << rowElim);
  Debugcln(FSP,"colElim:" << colElim);
  //Debugcln(FSP,"xElim");


  Int nullity;  // dim of null-space  (=id)
  Real K2;      // condition number   (=id)

  bIsZero = equals(Vector(vectorRange(bred,Range(0,Nred))),zeroVector(Nred),small); // is bred completely zero? (=bcheck)
  if (bIsZero) {
    Debugcln(FSP,"A is completely restricted");
    Matrix Asub(Nred, Mred); // submatrix of Ared (=id)
    Asub = matrixRange(Ared,Range(0,Nred), Range(0,Mred) );
Debugcln(Tmp,"Nred=" << Nred << " Mred=" << Mred << " Asub=\n" << Asub);

    Matrix n( Math::nullSpace(Asub, nullity, K2) ); // null-space vectors (=id)

    // set solution vectors for a completely restricted system (cols of g from rows of n)
    const Int Span2 = g.size1()-numSpecialgs; // (=SPAN2)
    for(Int gc=0; gc < Mred; gc++) {
      for(Int gr=0; gr < Span2-1; gr++)
        g(gr,gc) = n(gc,gr);
      g(Nred,gc) = 0;
    }

    systemComplete=true;
    status = Restricted;

  }
  else { // !bIsZero

    if (Nred != N) {
      Debugcln(FSP,"A is restricted for " << Nred << " out of " << N << " vector components");
    }

    const Int Span2 = g.size1()-numSpecialgs; // (=SPAN2)
    Matrix block(Span2, Nred); // location of blocked columns for each soln (=id)
    block = zeroMatrix(Span2,Nred);
    Matrix Asqr(Nred, Nred); // (=id)

    Int nextToFind = 0; // vector being searched for (=NextToFind)

    //
    // find first block pattern
    //

    // initialize the blocking positions for the first solution vector
    for(Int c=0; c<Nred; c++) block(0,c) = c;
Debugcln(DJ,"initialized block=" << matrixRow(block,0) << " Mred=" << Mred << " Nred=" << Nred);
    // The IKORv2.0 code didn't test for k<Nred here, which caused tests for block(0,k) with k out of range
    //  - which probably (by luck) didn't == i, hence didn't cause any further writing into Asqr
    for(Int k=0, i=0; (i<Mred) && (k<Nred); i++) {
      if (block(0,k) == i) {
        for(Int r=0; r<Nred; r++)
          Asqr(r,k) = Ared(r,i);
        k++;
      }
    }
    Matrix n( Math::nullSpace(Asqr, nullity, K2) ); // null-space vectors (=id)

    //Debugcln(FSP,"block:\n" << block << "Asqr:\n" << Asqr);
    //Debugcln(FSP,"n:\n" << n);
    //Debugcln(FSP,"nullity=" << nullity << " K2=" << K2);


    //Debugcln(FSP,"block:\n" << block << "Asqr:\n" << Asqr);
    //Debugcln(FSP,"n:\n" << n);
    // loop until an acceptable well-conditioned first solution found
    while ( (nullity!=0) && (block(0,0) < N) ) {

      SInt pos = SInt(Nred)-1;
      Assert(pos < SInt(block.size2()) );
      while (block(0,pos) == Span2-1+pos) { pos--; Assert(pos>0); }
      block(0,pos)++;
      Int move=pos+1;
      while (move < Nred) {
        block(0,move) = block(0,move-1)+1;
        move++;
      }

      // The IKORv2.0 code didn't test for k<Nred here, which caused tests for block(0,k) with k out of range
      //  - which probably (by luck) didn't == i, hence didn't cause any further writing into Asqr
      for(Int k=0, i=0; (i<Mred) && (k<Nred); i++) {
        if (block(0,k) == i) {
          for(Int r=0; r<Nred; r++)
            Asqr(r,k) = Ared(r,i);
          k++;
        }
      }
Debugcln(FSP,"trying initial block=" << matrixRow(block,0) );
      if (block(0,0) <= Nred)
        n.reset( Math::nullSpace(Asqr, nullity, K2) ); // calc n, nullity, K2

      //Debugcln(FSP,"block:\n" << block << "Asqr:\n" << Asqr);
      //Debugcln(FSP,"nullity=" << nullity << " K2=" << K2);
    }
Debugcln(DJ,"found suitable first block=" << matrixRow(block,0) );




    IVector tackon = zeroIVector(Mred-Nred); // columns are indexed for effic, soln finding (=Tackon)  //!!! changed tackon[] to Mred-Nred from M-N
    for(Int k=0, I=0, i=0; i<Mred; i++) {
      bool colIncluded = false;
      if (k<Nred)
        if (block(0,k)==i)
          colIncluded=true;
      if (!colIncluded) { // not in block, add it to tackon
        tackon[I]=i;
        I++;
      }
      else
        k++; // otherwise skip to next
    }
Debugcln(DJ,"tackon=" << tackon);

    // calculate a new g vector for a given square blocking pattern
    Vector gRow(matrixRow(g,nextToFind)); // select out rows[nextToFind]
    Vector blockRow(matrixRow(block,nextToFind));
    blockColFindX(gRow, tackon, blockRow, bred, Ared, Mred, Nred);
    matrixRow(g,nextToFind) = gRow;       // copy them back
    matrixRow(block,nextToFind) = blockRow;


    Debugcln(FSP,"first solution\nblock:\n" << matrixRow(block,0) << "\nAsqr:\n" << Asqr);
    if (Nred==Mred)
      systemComplete=true;
    else
      restOfSoln(M, N, Mred, Nred, nextToFind, block, g, bred, Ared, tackon, firstOK);

    Debugcln(FSP,"block:\n" << block << "solutions:\n" << g);

  } // end else !bIsZero


  // check if the eliminated b elements in bred can be
  //  produced from the g vectors and the reduced A matrix
  //   i.e. check that b is in the range of A
  bool binRange=false; // is b in range of A? (=binrange)
  if (!systemComplete) {
    Debugln(FSP,"System did not complete.  Non-fatal; continuing.");
  }
  else
    binRange = checkRange(A,b,g,rowElim);


  // check that the original b was not all zeros (special case)
  String reason;
  Assert(b.size()==N);
  bIsZero = equals(b,zeroVector(N),small);

  if (binRange && (!bIsZero) && (status != Restricted)) {

    rebuildgs(g, M, N, Mred, Nred, colElim, rowElim, specialg, numSpecialgs); // modifies g
    status = Complete;

    // the original IKOR2.0 code generated the beta constraints for rows eleminated due
    //  to dependency here.  That has been seperated out into the InverseKinematicsSolver
    //  (e.g. IKOR)
  }
  else {
    if (status != Restricted) {
      status = NotComplete;
      if (!bIsZero)
        reason += ": b is not in the range of A";
      else
        reason += ": special case - b is the zero vector; constrained optimization must be used";
    }
  }


  if (status == NotComplete)
    throw std::invalid_argument(Exception(String("FSP method unable to complete solution")+reason));

  // return the indices of rows that were eliminated due to dependency
  dependentRowsEliminated.clear();
  for(Int r=0; r<N; r++)
    if (rowElim[r] == RowEliminated_Dependent)
      dependentRowsEliminated.push_back(r);

  // return a M x span matrix of column vectors. (transpose it)
  Matrix gs(M,Mred-Nred+1+numSpecialgs);
  for(Int r=0; r<gs.size1(); r++)
    for(Int c=0; c<gs.size2(); c++)
      gs(r,c) = g(c,r);

  return gs;
}




bool IKORFullSpaceSolver::dependency(const Matrix& A, const base::IVector& slRow, Int Nred, Int first, Int second) const
{
  bool stop=false;
  Real devidor = 0;
  Int i=0;
  while ((i<Nred) && !stop) {
    if ( isSmall(A(slRow[i], first)) && isSmall(A(slRow[i], second)) )
      i++;
    else if ( (!isSmall(A(slRow[i], first))) && (!isSmall(A(slRow[i], second))) ) {
      if (devidor==0) {
        devidor = ( A(slRow[i],first) / A(slRow[i],second) );
        i++;
      }
      else if (isSmall(A(slRow[i], first) - (A(slRow[i],second)*devidor)))
        i++;
      else
        stop = true;
    }
    else
      stop = true;

  } // end while

  return !stop;
}



bool IKORFullSpaceSolver::checkRange(const Matrix& A, const Vector& b, const Matrix& g, const base::IVector& rowElim)
{
  for(Int i=0; i<A.size1(); i++) {
    if (rowElim[i] == RowEliminated_Dependent) {
      for(Int j=0; j<g.size1();j++) {
        Real checkValue = 0.0;
        for(Int k=0; k<A.size2();k++) checkValue += A(i,k) * g(j,k);
        if ( Math::abs(b[i] - checkValue) > 0.01) return false;
      }
    }
  }
  return true;
}






void IKORFullSpaceSolver::reduceA(const Matrix& A, Matrix& Ared,
                                  const Vector& b, Vector& bred,
                                  const Int M, const Int N,
                                  Int& Mred, Int& Nred,
                                  IVector& colElim, IVector& rowElim,
                                  Matrix& specialg, Int& numSpecialgs)
{
  // initialize
  Nred = N; // number of rows in the reduced A
  Mred = M; // number of columns in the reduced A
  numSpecialgs = 0;
  rowElim = zeroIVector(N);
  colElim = zeroIVector(M);

  IVector slRow(N); // Still-Left-Row    - stores remaining non eliminated rows in sequence (=SLRow)
  IVector slCol(M); // Still-Left-Column - stores remaining non eliminated columns in sequence (=SLCol)
  for(Int i=0; i<N; i++) slRow[i]=i;
  for(Int i=0; i<M; i++) slCol[i]=i;

  Vector xElim(M); // Contains strict values for reduced dq's (=Xelim)
  xElim = zeroVector(M);

  Vector btemp(b);

  bool doneRed = false; // Keeps track if reduction is done, loop has executed without any reduction (=DoneRed)

  // start reduction
  while (!doneRed) {

    doneRed = true;
    bool stillChecking = true; // flag to mark when all nonredundancies are gone (=StillChecking)

    // Check each row for the number of nonzero elements.  If only one
    // element is nonzero, solve for the joint space motion, and
    // modify the b vector so that the appropriate row and column
    // can be eliminated from the A.  After a row is eliminated
    // the remaining A must be rechecked for any new restrictions

    while (stillChecking) {

      Int lastNred = Nred;
      for(Int i=0; i<Nred; i++) {
        SInt j=-1;
        Int restriction = 0; // num of joints which contrib to a work space d.o.f.!!! (=Restriction)

        // check for non-zero row elements
        Int nonZeroCol=0; // Column which has nonzero element for given row (=NonZeroCol)
        while ( (j < SInt(Mred-1)) && (restriction < 2) ) {
          j++;
          if (!isSmall(A(slRow[i],slCol[j])) ) {
            restriction++;
            nonZeroCol = j;
          }
        }

        // if a row only has one nonzero element, eliminate it from the A
        if ( (restriction==1) && (rowElim[slRow[i]] != Eliminated_ZeroOrRestriction) ) {

          xElim[slCol[nonZeroCol]] = btemp[slRow[i]]/A(slRow[i],slCol[nonZeroCol]);

          for(Int k=0; k<Nred; k++)
            btemp[slRow[k]] -= A(slRow[k],slCol[nonZeroCol]) * xElim[slCol[nonZeroCol]];


          colElim[slCol[nonZeroCol]] = Eliminated_ZeroOrRestriction;
Debugln(DJ,"Eliminating column " <<  slCol[nonZeroCol] << " due to restriction");
//Debugln(DJ,"1: reducing Mred from " << Mred+1 << " to " << Mred << " (zero or restriction)");
          for(Int j=nonZeroCol; j<(Mred-1); j++) slCol[j] = slCol[j+1];
          Mred--;

          rowElim[slRow[i]] = Eliminated_ZeroOrRestriction;
Debugln(DJ,"eliminated row " << slRow[i] << " due to restriction");
          for(Int j=i; j<(Nred-1); j++) slRow[j] = slRow[j+1];
          Nred--;
//Debugln(DJ,"2: reducing Nred from " << Nred+1 << " to " << Nred << " (zero or restriction)");

        }
        else
          if ( (restriction==0) && (rowElim[slRow[i]] != Eliminated_ZeroOrRestriction) ) {
            // row of all zeros also eliminated
            rowElim[slRow[i]] = Eliminated_ZeroOrRestriction;
Debugln(DJ,"row " << slRow[i] << " eliminated (all zeros)");
            for(Int j=i; j<(Nred-1); j++) slRow[j] = slRow[j+1];
            Nred--;
//Debugln(DJ,"3: reducing Nred from " << Nred+1 << " to " << Nred << " (row of all zeros)");
          }

      } // for

      if (Nred == lastNred)
        stillChecking = false;

    } // end while (stillChecking)


    // check for columns of zeros
    for (Int i=0; i<Mred; i++) {
      Int j=0;
      while ( (j<Nred) && isSmall(A(slRow[j],slCol[i])) ) j++;

      if (j == Nred) {
        colElim[slCol[i]] = Eliminated_ZeroOrRestriction;
Debugln(DJ,"4: reducing Mred from " << Mred+1 << " to " << Mred << " (column " << slCol[i] << " of all zeros)");
        for(Int j=i; j<(Mred-1); j++) slCol[j] = slCol[j+1];
        Mred--;
        i--;
      }
    } // end for


    // check for dependent rows
    if (Mred != 0) {

      Matrix n;
      Matrix Atrans(Mred,Mred);

      for(Int i=0; i<Nred; i++)
        for(Int j=0; j<Mred; j++)
          Atrans(j,i) = A(slRow[i], slCol[j]);
      for(Int i=Nred; i<Mred; i++)
        for(Int j=0; j<Mred; j++)
          Atrans(j,i) = 0;
//Debugln(FSP,"Atrans:" << Atrans.size1() << "x" << Atrans.size2());
      Int nullity;
      Real K2;
      n.reset( Math::nullSpace(Atrans, nullity, K2) );
      nullity += Nred-Mred;
//Debugln(FSP,"nullity=" << nullity);
      // !!! IKORv2.0 says:
      // This is where the b's should be checked and determined if they
      // are dependent as well.  If they are not, an error message
      // should be displayed and the system should stop.
      // This is probably the best place to do this, instead of at the
      // bottom of solution_generator(), since it has the information.

//Debugln(FSP,"slRow.size()=" << slRow.size());//!!!
      Int nullCount=0; // (=nul_count)
      while (nullCount < nullity) {
        Int i=Mred-Nred + nullCount;
        Int j=0;
        while ( (j!=n.size1()) && isSmall(n(j,i)) ) j++;

        if ( (rowElim[slRow[j]] != Eliminated_ZeroOrRestriction) && (j != n.size1()) ) {
          rowElim[slRow[j]] = RowEliminated_Dependent;
          for(Int k=j; j<(Nred-1); k++) {
//Debugcln(FSP,"k=" << k);
            slRow[k] = slRow[k+1]; //!!! potential index oob here
          }
          Nred--;
Debugln(DJ,"5: reducing Nred from " << Nred+1 << " to " << Nred << " (dependent row)");
        }

        nullCount++;

      } // end while

    } //  end if (Mred!=0) // check for dependent rows



    // check for Special Case 1
    bool stop;
    {
      IVector nonZero(M); // stores values of nonzero col (=Nonzero)
      nonZero = zeroIVector(M);
      Int nonZeroCol;

      stop=false;
      for(SInt i=0; i< SInt(Nred); i++) { // for each row (& corresponding elt of btemp)

        if (isSmall(btemp[slRow[i]])) {
          nonZeroCol = 0;
          SInt j=0;
          stop = false;

          while ( (j < (SInt(Mred)-1)) && !stop) {
            if (!isSmall(A(slRow[i], slCol[j]))) {
              Int k=j+1;
              while (!stop && (k<Mred)) {
                if (!isSmall(A(slRow[i], slCol[k]))) {
                  if (dependency(A, slRow, Nred, slCol[j], slCol[k]))
                    k++;
                  else
                    stop=true;
                }
                else
                  k++;

              } // end while (!stop...)
              nonZero[nonZeroCol] = slCol[j];
              nonZeroCol++;
              j++;
            } // end if (!isSmall.. )
            else
              j++;

          } // end while ( j < ...)

          if (!isSmall(A(slRow[i], slCol[Mred-1]))) {
            nonZero[nonZeroCol] = slCol[Mred-1];
            nonZeroCol++;
          }

          if (nonZeroCol==1)
            stop = true;

          if (!stop) { // discovered a special-case 1
            doneRed = false;
Debug(DJ,"Discovered specialCase 1 in row " << i << " eliminating cols:");
            for(Int r=0; r<nonZeroCol; r++) {
              colElim[nonZero[r]] = ColEliminated_SpecialCase1;
Debugc(DJ," " << nonZero[r]);
            }
Debugcln(DJ,"");

            for(Int count=0, count2=0, position=0; position<Mred; position++) {
              if (slCol[position] != nonZero[count2]) {
                slCol[count] = slCol[position];
                count++;
              }
              else
                count2++;
            }


            // Construct new g Matrix
            Assert(nonZeroCol > 0);
            Matrix Aelim(1,nonZeroCol);
            for(Int r=0; r<nonZeroCol; r++)
              Aelim(0,r) = A(slRow[i], nonZero[r]);
            Int nullity;
            Real K2;
//	    Matrix n; //(nonZeroCol, nonZeroCol-1/*?*/);



//Debugln(FSP,"b n dim " << n.size1() << "x" << n.size2());//!!!
            Matrix n = Math::nullSpace(Aelim, nullity, K2);
/* added by DJ (perhaps should be nonZeroCol, not nzc-1)
            // pad n with 0s in case it is smaller than expected
            Matrix newn(nonZeroCol,nonZeroCol-1);
            newn = zeroMatrix(nonZeroCol,nonZeroCol-1);
            matrixRange(newn,Range(0,n.size1()),Range(0,n.size2())) = n;
            n.reset(newn);
*/
            Mred -= nonZeroCol;
Debugln(DJ,"6: SC1: reducing Mred from " << Mred+nonZeroCol << " to " << Mred);

            //!!! appropriate
            if (Mred == 0)
              throw std::invalid_argument(Exception("FSP method unable to complete solution: A reduced to 0 columns (Mred==0)"));


            rowElim[slRow[i]] = RowEliminated_Special;
            for(Int r=i; r<(Nred-1); r++) slRow[r] = slRow[r+1];
            Nred--;
Debugln(DJ,"7: SC1: reducing Nred from " << Nred+1 << " to " << Nred);

            //!!! appropriate?
            if (Nred == 0)
              throw std::invalid_argument(Exception("FSP method unable to complete solution: A reduced to 0 rows (Nred==0)"));

//Debugln(FSP,"specialg dim " << specialg.size1() << "x" << specialg.size2());//!!!
//Debugcln(FSP,"numSpecialgs=" << numSpecialgs << " M=" << M << " nonZeroCol=" << nonZeroCol);
//Debugcln(FSP,"nonZero.size()=" << nonZero.size());
//Debugln(FSP,"n dim " << n.size1() << "x" << n.size2());//!!!
            for(Int m=0; m<(nonZeroCol-1); m++) {

              for(Int r=0; r<M; r++)
                specialg(numSpecialgs, r) = 0;

              for(Int r=0; r<nonZeroCol; r++) {
//Debugcln(FSP,"numSpecialgs=" << numSpecialgs << " nonZero[r]=" << nonZero[r]); //!!!
                specialg(numSpecialgs, nonZero[r]) = n(r,m);
              }
              numSpecialgs++;
            }

          }

        } // end if (.. < small)

      } // end for each row
dump_A();
dump_btemp();
    } // end Special Case 1



    // check for Special Case 2
    {
      bool stop2=false;
      Vector constant(N);
      constant = zeroVector(N);
      Int reference=0; // reference position for row
      IVector classA(M); // classificationA columns
      classA = zeroIVector(M);
      Int classANum=0; // number of ClassificationA columns (=ClassANum)

      SInt i=0;
      while ( (i<SInt(Nred)-1) && !stop2 ) {

        SInt j=i+1;
        while ( (j < SInt(Nred)) && !stop2) {

          stop=false;
          bool flag=false; // beta undefined flag (=Flag)
          Real beta=0; // (=beta)

          if (!isSmall(btemp[slRow[i]]))
            beta = btemp[slRow[j]] / btemp[slRow[i]];
          else
            flag = true;

          classANum=0;
          Int k=0;
          reference=i;

          while ( !stop && (k<Mred) ) {
            bool goIn = false; // "Go in" for entance into ClassA (=Goin)

            if (isSmall(A(slRow[i], slCol[k]))) {
              if (!flag) goIn = true;
            }
            else {
              if (!isSmall( A(slRow[j],slCol[k]) / A(slRow[i],slCol[k]) - beta ) )
                goIn=true;
              if (flag) goIn=true;
            }

            if (goIn) {

              classA[classANum] = slCol[k];
              if (classANum==0) {

                if (isSmall(A(slRow[reference],classA[0]))) {
                  reference=j;
                  if (isSmall(A(slRow[reference],classA[0]))) //!!! redundant test?
                    stop = true;
                }

                if (!stop)
                  for(Int r=0; r<Nred; r++)
                    constant[r] = A(slRow[r],classA[0]) / A(slRow[reference],classA[0]);

              } // end if (classANum==0)
              else { // i.e. classANum != 0
                if (isSmall(A(slRow[reference],classA[classANum])))
                  stop = true;
                else {
                  Int r=0;
                  while ( (r<Nred) && !stop) {
                    if (!isSmall( (A(slRow[r],classA[classANum]) / A(slRow[reference],classA[classANum]) ) - constant[r]))
                      stop=true;
                    r++;
                  } // end while
                }
              } // end else (classANum != 0)

              classANum++;

            } // end if (goIn)
            k++;

          } // end while (!stop && (k<Mred)
          if (!stop) stop2=true;
          j++;

        } // end while (j < Nred) && !stop2
        i++;

      } // end while (i < Nred-1) && !stop2


      if (stop2) {
Debugln(DJ,"Discovered specialCase 2");

        doneRed=false;

        // construct new g matrix
        Matrix Aelim(1,classANum);
        Matrix n(classANum, classANum-1);
        for(Int p=0; p<classANum; p++)
          Aelim(0,p) = A(slRow[reference], classA[p]);
        Int nullity;
        Real K2;
        n = Math::nullSpace(Aelim, nullity, K2);

        for(Int p=0; p<(classANum-1); p++) {
          for(Int r=0; r<M; r++)
            specialg(numSpecialgs,r)=0;
          for(Int r=0; r<classANum; r++)
            specialg(numSpecialgs, classA[r]) = n(r,p);
          numSpecialgs++;
Debugcln(DJ,"new soln: " << Vector(matrixRow(specialg, numSpecialgs-1)) );
        } // end for p

        Nred--;
//Debugcln(DJ,"8: SC2: reducing Nred from " << Nred+1 << " to " << Nred);
        //!!! appropriate?
        if (Nred == 0)
          throw std::invalid_argument(Exception("FSP method unable to complete solution: A reduced to 0 rows (Nred==0)"));

        rowElim[slRow[reference]] =  RowEliminated_Special;
Debugc(DJ,"eliminated row " << slRow[reference] << " and cols:");
        for(Int r=0; r<classANum; r++) {
          colElim[classA[r]] = ColEliminated_SpecialCase2;
Debugc(DJ," " << classA[r]);
        }
Debugcln(DJ,"");
        for(Int r=reference; r<Nred; r++) slRow[r] = slRow[r+1];

        for(Int count=0, count2=0, position=0; position<Mred; position++)
          if (slCol[position] != classA[count2]) {
            slCol[count] = slCol[position];
            count++;
          }
          else
            count2++;

        //!!! appropriate?
        if ((SInt(Mred) - SInt(classANum)) <= 0)
          throw std::invalid_argument(Exception("FSP method unable to complete solution: A reduced to 0 columns (Mred==0)"));

        Mred -= classANum;
//Debugln(DJ,"9: SC2: reducing Mred from " << Mred+classANum << " to " << Mred);
      } // end if (stop2)

    } // end Special Case 2


  } // end while (!doneRed)




  // store the reduced A in the variable Ared, and the modified b in bred
  if ((Nred > 0) && (Mred > 0)) {
    Ared.resize(Nred, Mred);
    bred.resize(Nred);
    SInt j=-1;
    for(Int i=0; i<N; i++)
      if (rowElim[i] == NotEleminated) {
        j++;
        bred[j] = btemp[i];
        SInt m=-1;
        for(Int k=0; k<M; k++)
          if (colElim[k] == NotEleminated) {
            m++;
            Ared(j,m) = A(i,k);
          }
      }
  }
  else
    throw std::runtime_error(Exception("FSP method error: The A matrix has been reduced to nothing"));

}



///  calculates a g vector for a given square blocking pattern.
void IKORFullSpaceSolver::blockColFindX(Vector& g,
                                        const IVector& tackon, const Vector& block,
                                        const Vector& b, const Matrix& A,
                                        const Int Mred, const Int Nred)
{
  Matrix Atemp(Nred, Nred);
  Vector btemp(Nred);
  Vector blocktemp(Nred);

  // the columns blocked need to be listed from smallest to largest
  btemp=b;
  blocktemp=block;

  for(Int i=(Nred-1); i>0; i--)
    for(SInt j=i-1; j>=0; j--)
      if (blocktemp[i] < blocktemp[j])
        base::swap(blocktemp[i], blocktemp[j]);
Debugln(DJ,"blocktemp=" << blocktemp);
  // !!! IKORv2 has an I var in this loop, it doesn't appear to be used (??)
  for(Int k=0, i=0; (i<Mred) && (k<Nred); i++) { //!!! added k<Nred
    if (blocktemp[k] == i) {
      matrixColumn(Atemp,k) = matrixColumn(A,i);
      k++;
    }
  }
Debugln(DJ,"Atemp=\n" << Atemp);
  //Debugcln(FSP,"asqr=\n" << Atemp);

  Vector gtemp( Math::inverse(Atemp)*btemp );

  Debugcln(FSP,"gvector=\n" << gtemp);

  // Now add a zero to where column was blocked
  for (Int i=0; i<Nred; i++)
    g[Int(blocktemp[i])] = gtemp[i];

  Assert(Mred-Nred <= tackon.size());
  for(Int j=0; j<Mred-Nred; j++) {
    if (tackon[j] < g.size()) g[tackon[j]]=0; //!!! added if (..<..)
  }

}






void IKORFullSpaceSolver::restOfSoln(const Int M, const Int N, const Int Mred, const Int Nred,
                                     Int nextToFind,
                                     Matrix& block, Matrix& g,
                                     const Vector& bred, const Matrix& Ared,
                                     const base::IVector& tackon, base::IVector& firstOK)
{
  Debugln(FSP,"Entering RestOfSoln()");
  nextToFind++; // looking for next soln

  Int X, Y; // Position markers, keeping track of the current column to be subbed in for and which one to sub in (=id)

  X = nextToFind-1;
Debugcln(FSP,"g=\n" << g);

  // Change firstOK if a previously 0 value of g for a column in first
  // blocking pattern has become nonzero
  for(Int i=0; i<Nred; i++)
    if ( (!firstOK[i]) && (!isSmall(g(nextToFind-1, Int(block(nextToFind-1,i))))) )
      firstOK[i] = Int(true);

  IVector changed(Nred); // temp vector for firstOK, passed to recursive call (=Changed)
  changed = vectorRange(firstOK,Range(0,Nred));
  Debugcln(FSP,"changed=" << changed);

  while ( (X < Mred-Nred) && (!systemComplete) ) {

    Debugcln(FSP,"X=" << X << " next soln to find is " << nextToFind);

    // Set up next Asub comparing the next remaining column to be subbed
    // in with the blocking pattern for that level of recursion
    Matrix Asub(Nred,Nred+1);
    for(Int i=0; i<Nred; i++)
      for(Int r=0; r<Nred; r++)
        Asub(r,i) = Ared(r, Int(block(nextToFind-1,i)) );
    for(Int r=0; r<Nred; r++)
      Asub(r,Nred) = Ared(r,tackon[X]);

    Int nullity; // dim of null-space  (=id)
    Real K2;     // condition number   (=id)
    Matrix n( Math::nullSpace(Asub, nullity, K2) ); // null-space vectors (=id)

    Y=0;
    // Loop which compares column to be subbed in with current blocking
    // pattern for dependency, if dependent it completes swap and valid
    while ( (Y < Nred) && (!systemComplete) ) {

      Debugcln(FSP,"Y=" << Y << " X=" << X);
      Debugcln(FSP, "compare column " << tackon[X] << " with " << Int(block(nextToFind-1,Y)) );
      Debugcln(FSP, " it is " << Math::abs(n(Y,0)) << " and firstOK[Y]=" << firstOK[Y]);

      if ( (!isSmall(n(Y,0))) && (firstOK[Y]) ) {

        Debugcln(FSP,"*************** " << nextToFind << " ****************");

        Debugcln(FSP,"** block=\n" << block);
        Debugcln(FSP,"** tackon=" << tackon);
        Debugcln(DJ, "** X=" << X << " Y=" << Y);

        IVector newTack(M-N); // temp vector for tackon, passed in recursive call
        for(Int i=0; i<Nred; i++) block(nextToFind,i) = block(nextToFind-1,i);
        newTack[0] = Int(block(nextToFind,Y));
        block(nextToFind,Y) = tackon[X];
        for(Int i=0; i<X; i++) newTack[i+1] = tackon[i];
        for(Int j=X+1; j<Mred-Nred; j++)
          newTack[j] = tackon[j];

        Debugcln(FSP,"** newTack=" << newTack);
        Debugcln(FSP,"** block=\n" << block);
        Debugcln(FSP,"** g=\n" << g);

        Vector gRow(matrixRow(g,nextToFind)); // select out rows[nextToFind]
        Vector blockRow(matrixRow(block,nextToFind));
        blockColFindX(gRow, newTack, blockRow, bred, Ared, Mred, Nred);
        matrixRow(g,nextToFind) = gRow;       // copy them back
        matrixRow(block,nextToFind) = blockRow;

        Debugln(FSP,"** block=\n" << block);
        Debugcln(FSP,"** g=\n" << g);

        Debugcln(FSP," it will be g(nextToFind,block(nextToFind,Y)=" << Math::abs(g(nextToFind,Int(block(nextToFind,Y)))));


        // Check to ensure the newly created g vector is independent
        // of previously created ones in branch
        if ( !isSmall(g(nextToFind,Int(block(nextToFind,Y)))) ) {
          if (nextToFind < (Mred-Nred)) {
            //Debugcln(FSP,"block:\n" << block);
            Debugcln(FSP,"solutions:\n" << g);

            // recursive call
            restOfSoln(M, N, Mred, Nred, nextToFind, block, g, bred, Ared, newTack, changed);
          }
          else {
            systemComplete=true;
          }
        } // endif abs(g...) > small


        if (!systemComplete)
          block(nextToFind,Y) = newTack[0];

      } // endif Y > small && ...

      Y++;

    } // while

    X++;

  } // while


}




void IKORFullSpaceSolver::rebuildgs(Matrix& g,
                                    const Int M, const Int N, const Int Mred, const Int Nred,
                                    const IVector& colElim, const IVector& rowElim,
                                    const Matrix& specialg, const Int numSpecialgs)
{
  Matrix gtemp(g);

  for(Int k=0, j=0; j<M; j++) {

    if (colElim[j] != NotEleminated) {
      for(Int i=0; i<g.size1(); i++) g(i,j) = 0;
    }
    else {
      for(Int i=0; i<g.size1(); i++) g(i,j) = gtemp(i,k);
      k++;
    }
  }

  // Addition of null vectors created by SpecialCase1
  for(Int i=(Mred-Nred+1); i<(Mred-Nred+1+numSpecialgs); i++)
    for(Int j=0; j<M; j++) {
      if (colElim[j] != NotEleminated)
        g(i,j) = specialg(i-Mred+Nred-1,j);
      else
        g(i,j) = g(0,j);
    }

}



