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

  $Id: Expression.cpp 1029 2004-02-11 20:45:54Z jungd $
  $Revision: 1.17 $
  $Date: 2004-02-11 15:45:54 -0500 (Wed, 11 Feb 2004) $
  $Author: jungd $

****************************************************************************/

#include <base/Expression>

#include <base/Matrix>
#include <base/Math>
#include <base/ConstantExpression>
#include <base/VariableExpression>
#include <base/SumExpression>
#include <base/DifferenceExpression>
#include <base/NegateExpression>
#include <base/ProductExpression>
#include <base/QuotientExpression>
#include <base/SinExpression>
#include <base/CosExpression>


using base::Math;
using base::Matrix;
using base::Expression;
using base::ExpressionNode;
using base::ConstantExpression;
using base::VariableExpression;
using base::SumExpression;
using base::DifferenceExpression;
using base::NegateExpression;
using base::ProductExpression;
using base::QuotientExpression;
using base::SinExpression;
using base::CosExpression;



Expression::Expression()
{
  expr = ref<ExpressionNode>(NewObj ConstantExpression(0));
}

Expression::Expression(Real constant)
{
  expr = ref<ExpressionNode>(NewObj ConstantExpression(constant));
}

Expression::Expression(const Expression& e)
{
  expr = e.expr;
}

Expression::Expression(const String& exprString)
{
  Int pos=0;
  expr = expression(exprString,pos).expr;
}

  
  
base::Real Expression::evaluate(const Vector& params) const
{
  expr->resetCache();
  return expr->evaluate(params);
}

Expression Expression::differentiate( Expression withRespectTo ) const
{
  if ( withRespectTo.expr->opType() != ExpressionNode::Variable ) 
    throw std::invalid_argument(Exception("Must pass a simple variable expression, such as Expression::p[2]"));

  ref<VariableExpression> vexpr( narrow_ref<VariableExpression>(withRespectTo.expr) );
  return Expression(expr->differentiate(vexpr->index));
}

/// \todo add mergeCommonSubexpressions()
void Expression::simplify()
{
  expr = simplifyConstantExpressions(expr);
}

void Expression::operationCounts(Int& addsub, Int& multdiv, Int& trig) const
{
  expr->operationCounts(addsub,multdiv,trig);
}


base::String Expression::toString() const
{
  return expr->toString();
}

Expression Expression::VariableIndexer::operator[](Int i) const
{
  ref<ExpressionNode> expr(NewObj VariableExpression(i));
  return Expression(expr);
}


Expression::VariableIndexer Expression::p;





// parsing
bool Expression::peek(const String& s, Int pos, String next)
{
  if (pos + next.size() > s.size()) return false;
  
  return (s.find(next, pos) == pos);
}

inline bool isAlpha(String::value_type c)
{
  return ((c >= 'a') && (c <= 'z')) || ((c >= 'A') && (c <= 'Z'));
}

inline bool isNum(String::value_type c)
{
  return ((c >= '0') && (c <= '9'));
}

inline bool isAlphaNum(String::value_type c)
{
  return isAlpha(c) || isNum(c);
}


SInt Expression::index(const String& s, Int& pos)
{
  SInt sign=1;
  if (peek(s,pos,'-')) { // leading -ve?
    sign = -1;
    ++pos;
  }
  else
    if (peek(s,pos,'+')) ++pos; // leading +ve?
  
  Int v=0;
  while ( isNum(s[pos]) ) { 
    v = (10*v) + Int(s[pos]-'0');
    ++pos;
  }
  
  return SInt(sign*v);
}


Real Expression::real(const String& s, Int& pos)
{
  Real sign=1.0;
  if (peek(s,pos,'-')) { // leading -ve?
    sign = -1.0;
    ++pos;
  }
  else
    if (peek(s,pos,'+')) ++pos; // leading +ve?
  
  Real v=0;
  while ( isNum(s[pos]) ) { // whole part
    v = (10.0*v) + Int(s[pos]-'0');
    ++pos;
  }
  if (peek(s,pos,'.')) { // fraction part (optional)
    ++pos;
    Real m=0.1;
    while (isNum(s[pos])) {
      v = v + Real(Int(s[pos]-'0'))*m;
      m *= 0.1;
      ++pos;
    }
  }
  if (peek(s,pos,'e') || peek(s,pos,'E')) { // exponent part (optional)
    if (peek(s,pos+1,'-') || (peek(s,pos+1,'+'))) { // -ve or +ve required
      ++pos;
      Real esign = peek(s,pos,'-')?-1.0:1.0;
      ++pos;
      Real exp=0;
      while ( isNum(s[pos]) ) {
        exp = (10.0*exp) + Int(s[pos]-'0');
        ++pos;
      }
      v = v * Math::pow(10.0,esign*exp);
    }
  }

  return sign*v;
}

  
Expression Expression::expression(const String& s, Int& pos)
{
  Expression lhs = term(s,pos);
  while (peek(s,pos,'+') || peek(s,pos,'-')) {
    String::value_type op = s[pos++];
    Expression rhs = term(s,pos);
    if (op == '+')
      lhs = lhs + rhs;
    else
      lhs = lhs - rhs;
  }
  return lhs;
}


Expression Expression::term(const String& s, Int& pos)
{
  Expression lhs = prod(s, pos);
  while (peek(s,pos,'*') || peek(s,pos,'/')) {
    String::value_type op = s[pos++];
    Expression rhs = prod(s,pos);
    if (op == '*')
      lhs = lhs * rhs;
    else
      lhs = lhs / rhs;
  }
  return lhs;
}


Expression Expression::prod(const String& s, Int& pos)
{
  Expression e;
  if (peek(s,pos,'(')) { // '(' expression ')'
    ++pos;
    e = expression(s,pos);
    if (!peek(s,pos,')')) throw std::invalid_argument(Exception(String("expecting ')' but got '")+s[pos]+"' in expression"));
    ++pos;
  }
  else if (peek(s,pos,"p[")) { // 'p[' index ']'
    pos += 2;
    Int i = index(s,pos);
    if (!peek(s,pos,']')) throw std::invalid_argument(Exception(String("expecting ']' but got '")+s[pos]+"' after 'p[<index>' in expression"));
    ++pos;
    e = Expression::p[i];
  }
  else if (peek(s,pos,"cos(")) { // cos
    pos += 4;
    Expression arg = expression(s,pos);
    e = base::cos(arg);
    if (!peek(s,pos,')')) throw std::invalid_argument(Exception(String("expecting ')' but got '")+s[pos]+"' after 'cos(<expression>' in expression"));
    ++pos;
  }
  else if (peek(s,pos,"sin(")) { // sin
    pos += 4;
    Expression arg = expression(s,pos);
    e = base::sin(arg);
    if (!peek(s,pos,')')) throw std::invalid_argument(Exception(String("expecting ')' but got '")+s[pos]+"' after 'sin(<expression>' in expression"));
    ++pos;
  }
  else if (peek(s,pos,"tan(")) { // tan
    pos += 4;
    Expression arg = expression(s,pos);
    e = base::sin(arg) / base::cos(arg); 
    if (!peek(s,pos,')')) throw std::invalid_argument(Exception(String("expecting ')' but got '")+s[pos]+"' after 'tan(<expression>' in expression"));
    ++pos;
  }
  else if (peek(s,pos,"pi")) { // 'pi' is an alias for Expression(consts::Pi)
    pos += 2;
    e = Expression(consts::Pi);
  }
  else if (peek(s,pos,'s')) { // 's' as an alias for p[0] (we already checked for 'sin..')
    ++pos;
    e = Expression::p[0];
  }
  else { // assume real
    Real v = real(s,pos);
    e = Expression(v);
  }
    
  return e;
}



// Member Operators/Operations
Expression& Expression::operator+=(const Expression& e)
{
  expr = ref<ExpressionNode>(NewObj SumExpression(expr,e.expr));
  return *this;
}

Expression& Expression::operator-=(const Expression& e)
{
  expr = ref<ExpressionNode>(NewObj DifferenceExpression(expr,e.expr));
  return *this;
}

Expression& Expression::operator*=(const Expression& e)
{
  expr = ref<ExpressionNode>(NewObj ProductExpression(expr,e.expr));
  return *this;
}

Expression& Expression::operator/=(const Expression& e)
{
  expr = ref<ExpressionNode>(NewObj QuotientExpression(expr,e.expr));
  return *this;
}

Expression& Expression::negate()
{
  expr = ref<ExpressionNode>(NewObj NegateExpression(expr));
  return *this;
}

Expression& Expression::sin()
{
  expr = ref<ExpressionNode>(NewObj SinExpression(expr));
  return *this;
}

Expression& Expression::cos()
{
  expr = ref<ExpressionNode>(NewObj CosExpression(expr));
  return *this;
}


void Expression::serialize(Serializer& s)
{
  // register instantiators for all the ExpressionNode classes for dynamic instantiation upon deserialization
  Serializable::registerSerializableInstantiator<ExpressionNode,SumExpression>(sumInstantiator);
  Serializable::registerSerializableInstantiator<ExpressionNode,DifferenceExpression>(differenceInstantiator);
  Serializable::registerSerializableInstantiator<ExpressionNode,ProductExpression>(productInstantiator);
  Serializable::registerSerializableInstantiator<ExpressionNode,QuotientExpression>(quotientInstantiator);
  Serializable::registerSerializableInstantiator<ExpressionNode,NegateExpression>(negateInstantiator);
  Serializable::registerSerializableInstantiator<ExpressionNode,ConstantExpression>(constantInstantiator);
  Serializable::registerSerializableInstantiator<ExpressionNode,VariableExpression>(variableInstantiator);
  Serializable::registerSerializableInstantiator<ExpressionNode,SinExpression>(sinInstantiator);
  Serializable::registerSerializableInstantiator<ExpressionNode,CosExpression>(cosInstantiator);
  s.comment(String("Symbolic expression: "+toString()));
  s.baseRef(expr,"expression");
}




// Simplification helper methods
ref<ExpressionNode> Expression::simplifyConstantExpressions(ref<ExpressionNode> expr)
{
  if (!expr) { Assert(expr); }

  ref<ExpressionNode> sexpr(expr); // (potentially) simpler expression

  if (sexpr->isBinaryOp()) { 
    ref<BinaryOpExpression> binExpr(narrow_ref<BinaryOpExpression>(sexpr));

    // recursively simplify subtree first
    binExpr->leftArg  = simplifyConstantExpressions(binExpr->leftArg);
    binExpr->rightArg = simplifyConstantExpressions(binExpr->rightArg);

    // Test for constness and zero (< base::epsilon)
    bool leftIsConst  = binExpr->leftArg->opType()  == ExpressionNode::Constant;
    bool rightIsConst = binExpr->rightArg->opType() == ExpressionNode::Constant;
    ref<ConstantExpression> leftConst;
    if (leftIsConst) leftConst = narrow_ref<ConstantExpression>(binExpr->leftArg);
    bool leftIsZero = leftIsConst?(Math::equals(leftConst->constValue,0)):false;
    ref<ConstantExpression> rightConst;
    if (rightIsConst) rightConst = narrow_ref<ConstantExpression>(binExpr->rightArg);
    bool rightIsZero = rightIsConst?(Math::equals(rightConst->constValue,0)):false;

    // if both left & right are constants, replace with evaluated constant
    //  or if only one is a constant and == 0, eliminate it (Sum/Diff) or replace
    //  whole expression with 0 (Prod/Div)

    if (binExpr->opType() == ExpressionNode::Sum) {

      if (leftIsConst) {
	if (rightIsConst)
	  sexpr = ref<ExpressionNode>(NewObj ConstantExpression(leftConst->constValue + rightConst->constValue));
	else
	  if (leftIsZero)
	    sexpr = binExpr->rightArg; // 0+x = x
      }
      else
	if (rightIsConst && rightIsZero)
	  sexpr = binExpr->leftArg; // x+0 = x

    } // end Sum
    else if (binExpr->opType() == ExpressionNode::Difference) {

      if (leftIsConst) {
	if (rightIsConst)
	  sexpr = ref<ExpressionNode>(NewObj ConstantExpression(leftConst->constValue - rightConst->constValue));
	else
	  if (leftIsZero)
	    sexpr = ref<ExpressionNode>(NewObj NegateExpression(binExpr->rightArg)); // 0-x = -x
      }
      else
	if (rightIsConst && rightIsZero)
	  sexpr = binExpr->leftArg; // x-0 = x

    } // end Difference
    else if (binExpr->opType() == ExpressionNode::Product) {

      bool leftIsOne       = leftIsConst?(Math::equals(leftConst->constValue,1)):false;
      bool leftIsMinusOne  = leftIsConst?(Math::equals(leftConst->constValue,-1)):false;
      bool rightIsOne      = rightIsConst?(Math::equals(rightConst->constValue,1)):false;
      bool rightIsMinusOne = rightIsConst?(Math::equals(rightConst->constValue,-1)):false;

      if (leftIsConst) {
	if (rightIsConst)
	  sexpr = ref<ExpressionNode>(NewObj ConstantExpression(leftConst->constValue * rightConst->constValue));
	else {
	  if (leftIsZero)
	    sexpr = ref<ExpressionNode>(NewObj ConstantExpression(0)); // 0*x = 0
	  else
	    if (leftIsOne)
	      sexpr = binExpr->rightArg; // 1*x = x
	    else
	      if (leftIsMinusOne) {
		if ( binExpr->rightArg->opType() == ExpressionNode::Negative) 
		  sexpr = narrow_ref<NegateExpression>( binExpr->rightArg )->arg; // -1 * -x = x
		else sexpr = ref<NegateExpression>( NewObj NegateExpression( binExpr->rightArg )); // -1*x = -x
	      }
	
	}
      } 
      else
	if (rightIsConst) {
	  if (rightIsZero)
	    sexpr = ref<ExpressionNode>(NewObj ConstantExpression(0)); // x*0 = 0
	  else
	    if (rightIsOne)
	      sexpr = binExpr->leftArg; // x*1 = x
	    else
	      if (rightIsMinusOne) {
		if ( binExpr->leftArg->opType() == ExpressionNode::Negative )
		  sexpr = narrow_ref<NegateExpression>( binExpr->leftArg )->arg; // -x * -1 = x
		else sexpr = ref<NegateExpression>( NewObj NegateExpression( binExpr->leftArg )); // x*-1 = -x
	      }
	}

    } // end Product
    else if (binExpr->opType() == ExpressionNode::Quotient) {

      bool rightIsOne = rightIsConst?(equals(rightConst->constValue,1)):false;

      if (leftIsConst) {
	if (rightIsConst)
	  sexpr = ref<ExpressionNode>(NewObj ConstantExpression(leftConst->constValue / rightConst->constValue));
	else {
	  if (leftIsZero)
	    sexpr = ref<ExpressionNode>(NewObj ConstantExpression(0)); // 0/x = 0
	}
	
      }
      else
	if (rightIsConst) {
	  if (rightIsZero) {
	    throw std::out_of_range(Exception("cannot divide by constant 0")); // x/0 = error
	  }
	  else
	    if (rightIsOne)
	      sexpr = binExpr->leftArg; // x/1 = x
	}

    } // end Quotient

  } // end BinaryOp
  else if (sexpr->isUnaryOp()) {
    ref<UnaryOpExpression> unaryExpr(narrow_ref<UnaryOpExpression>(sexpr));

    // recursively simplify subtree first
    unaryExpr->arg = simplifyConstantExpressions(unaryExpr->arg);

    // Test for constness 
    bool isConst  = unaryExpr->arg->opType() == ExpressionNode::Constant;
    ref<ConstantExpression> argConst;
    if (isConst) argConst = narrow_ref<ConstantExpression>(unaryExpr->arg);

    if (unaryExpr->opType() == ExpressionNode::Sine) {
      if (isConst)
	sexpr = ref<ExpressionNode>(NewObj ConstantExpression(Math::sin(argConst->constValue)));
    }
    else if (unaryExpr->opType() == ExpressionNode::Cosine) {
      if (isConst)
	sexpr = ref<ExpressionNode>(NewObj ConstantExpression(Math::cos(argConst->constValue)));
    }
    else if (unaryExpr->opType() == ExpressionNode::Negative) {
      ref<NegateExpression> nexpr( narrow_ref<NegateExpression>(unaryExpr) );
      if (nexpr->arg->opType() == ExpressionNode::Constant) {
	bool isZero = Math::equals(ref<ConstantExpression>(narrow_ref<ConstantExpression>(nexpr->arg))->constValue,0);
	if (isZero)
	  sexpr = nexpr->arg;
      }
      else if (nexpr->arg->opType() == ExpressionNode::Negative) {
	sexpr = narrow_ref<NegateExpression>(nexpr->arg)->arg;
      }
    } // end Negative

  } // end UnaryOp

  return sexpr;
}




void base::simplify( ExpressionMatrix& m ) 
{
  for ( Int r=0; r < m.size1(); r++ ) {
    for ( Int c=0; c < m.size2(); c++ ) {
      m(r,c).simplify();
    }
  }
  return;
}



base::Matrix base::evaluate( const ExpressionMatrix& m, const Vector& params ) 
{
  Matrix em(m.size1(), m.size2());
  for ( Int r=0; r < m.size1(); r++ ) 
    for ( Int c=0; c < m.size2(); c++ ) 
      em(r,c) = m(r,c).evaluate(params);
  return em;
}


base::ExpressionMatrix base::toExpressionMatrix(const Matrix& m)
{
  ExpressionMatrix em(m.size1(), m.size2());
  for( Int r=0; r < m.size1(); r++ ) 
    for ( Int c=0; c < m.size2(); c++ ) 
      em(r,c)=m(r,c);
  return em;
}
