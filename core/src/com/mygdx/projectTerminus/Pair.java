/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package com.mygdx.projectTerminus;

/**
 *
 * @author Shane
 */
public class Pair<L, R> {
    private L left;
    private R right;
    
    public Pair(){}
    
    public Pair(L leftObj, R rightObj)
    {
        left = leftObj;
        right = rightObj;
    }
    
    public L getLeft() {return left;}
    public void setLeft(L newLeft) {left = newLeft;}
    
    public R getRight() {return right;}
    public void setRight(R newRight) {right = newRight;}
    
    @Override
    public String toString()
    {
        return "Left: " + left.toString() + ", Right: " + right.toString();
    }
}
