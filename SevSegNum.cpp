#include "SevSegNum.h"


void printNum0on() 
{
    digitalWrite(SevenSegA, HIGH);    // turn the LED On by making the voltage LOW
    digitalWrite(SevenSegB, HIGH);
    digitalWrite(SevenSegC, HIGH);
    digitalWrite(SevenSegD, HIGH);
    digitalWrite(SevenSegE, HIGH);
    digitalWrite(SevenSegF, HIGH);
}

void printNum0off()
{
    digitalWrite(SevenSegA, LOW);    // turn the LED off by making the voltage LOW
    digitalWrite(SevenSegB, LOW);
    digitalWrite(SevenSegC, LOW);
    digitalWrite(SevenSegD, LOW);
    digitalWrite(SevenSegE, LOW);
    digitalWrite(SevenSegF, LOW);
}

void printNum1on()
{
    digitalWrite(SevenSegB, HIGH);
    digitalWrite(SevenSegC, HIGH);
}

void printNum1off()
{
    digitalWrite(SevenSegB, LOW);
    digitalWrite(SevenSegC, LOW);
}

void printNum2on()
{
    digitalWrite(SevenSegA, HIGH);
    digitalWrite(SevenSegB, HIGH);
    digitalWrite(SevenSegD, HIGH);
    digitalWrite(SevenSegG, HIGH);
    digitalWrite(SevenSegE, HIGH);
}

void printNum2off()
{
    digitalWrite(SevenSegA, LOW);
    digitalWrite(SevenSegB, LOW);
    digitalWrite(SevenSegD, LOW);
    digitalWrite(SevenSegG, LOW);
    digitalWrite(SevenSegE, LOW);
}

void printNum3on()
{
    digitalWrite(SevenSegA, HIGH);
    digitalWrite(SevenSegB, HIGH);
    digitalWrite(SevenSegC, HIGH);
    digitalWrite(SevenSegD, HIGH);
    digitalWrite(SevenSegG, HIGH);
}

void printNum3off()
{
    digitalWrite(SevenSegA, LOW);
    digitalWrite(SevenSegB, LOW);
    digitalWrite(SevenSegC, LOW);
    digitalWrite(SevenSegD, LOW);
    digitalWrite(SevenSegG, LOW);
}

void printNum4on()
{
    digitalWrite(SevenSegB, HIGH);
    digitalWrite(SevenSegC, HIGH);
    digitalWrite(SevenSegF, HIGH);
    digitalWrite(SevenSegG, HIGH);
}

void printNum4off()
{
    digitalWrite(SevenSegB, LOW);
    digitalWrite(SevenSegC, LOW);
    digitalWrite(SevenSegF, LOW);
    digitalWrite(SevenSegG, LOW);
}

void printNum5on()
{
    digitalWrite(SevenSegA, HIGH);
    digitalWrite(SevenSegC, HIGH);
    digitalWrite(SevenSegD, HIGH);
    digitalWrite(SevenSegF, HIGH);
    digitalWrite(SevenSegG, HIGH);
}

void printNum5off()
{
    digitalWrite(SevenSegA, LOW);
    digitalWrite(SevenSegC, LOW);
    digitalWrite(SevenSegD, LOW);
    digitalWrite(SevenSegF, LOW);
    digitalWrite(SevenSegG, LOW);
}

void printNum6on()
{
    digitalWrite(SevenSegA, HIGH);    // turn the LED On by making the voltage LOW
    digitalWrite(SevenSegC, HIGH);
    digitalWrite(SevenSegD, HIGH);
    digitalWrite(SevenSegE, HIGH);
    digitalWrite(SevenSegF, HIGH);
    digitalWrite(SevenSegG, HIGH);
}

void printNum6off()
{
    digitalWrite(SevenSegA, LOW);    // turn the LED On by making the voltage LOW
    digitalWrite(SevenSegC, LOW);
    digitalWrite(SevenSegD, LOW);
    digitalWrite(SevenSegE, LOW);
    digitalWrite(SevenSegF, LOW);
    digitalWrite(SevenSegG, LOW);
}

void printNum7on()
{
    digitalWrite(SevenSegA, HIGH);
    digitalWrite(SevenSegB, HIGH);
    digitalWrite(SevenSegC, HIGH);
}

void printNum7off()
{
    digitalWrite(SevenSegA, LOW);
    digitalWrite(SevenSegB, LOW);
    digitalWrite(SevenSegC, LOW);
}

void printNum8on()
{
    digitalWrite(SevenSegA, HIGH);    // turn the LED On by making the voltage LOW
    digitalWrite(SevenSegB, HIGH);
    digitalWrite(SevenSegC, HIGH);
    digitalWrite(SevenSegD, HIGH);
    digitalWrite(SevenSegE, HIGH);
    digitalWrite(SevenSegF, HIGH);
    digitalWrite(SevenSegG, HIGH);
}

void printNum8off()
{
    digitalWrite(SevenSegA, LOW);    // turn the LED On by making the voltage LOW
    digitalWrite(SevenSegB, LOW);
    digitalWrite(SevenSegC, LOW);
    digitalWrite(SevenSegD, LOW);
    digitalWrite(SevenSegE, LOW);
    digitalWrite(SevenSegF, LOW);
    digitalWrite(SevenSegG, LOW);
}

void printNum9on()
{
    digitalWrite(SevenSegA, HIGH);    // turn the LED On by making the voltage LOW
    digitalWrite(SevenSegB, HIGH);
    digitalWrite(SevenSegC, HIGH);
    digitalWrite(SevenSegD, HIGH);
    digitalWrite(SevenSegF, HIGH);
    digitalWrite(SevenSegG, HIGH);
}

void printNum9off()
{
    digitalWrite(SevenSegA, LOW);    // turn the LED On by making the voltage LOW
    digitalWrite(SevenSegB, LOW);
    digitalWrite(SevenSegC, LOW);
    digitalWrite(SevenSegD, LOW);
    digitalWrite(SevenSegF, LOW);
    digitalWrite(SevenSegG, LOW);
}

void printNumAon()
{
    digitalWrite(SevenSegA, HIGH);
    digitalWrite(SevenSegB, HIGH);
    digitalWrite(SevenSegC, HIGH);
    digitalWrite(SevenSegE, HIGH);
    digitalWrite(SevenSegF, HIGH);
    digitalWrite(SevenSegG, HIGH);
}

void printNumAoff()
{
    digitalWrite(SevenSegA, LOW);
    digitalWrite(SevenSegB, LOW);
    digitalWrite(SevenSegC, LOW);
    digitalWrite(SevenSegE, LOW);
    digitalWrite(SevenSegF, LOW);
    digitalWrite(SevenSegG, LOW);
}

void printNumBon()
{
    digitalWrite(SevenSegC, HIGH);
    digitalWrite(SevenSegD, HIGH);
    digitalWrite(SevenSegE, HIGH);
    digitalWrite(SevenSegF, HIGH);
    digitalWrite(SevenSegG, HIGH);
}

void printNumBoff()
{
    digitalWrite(SevenSegC, LOW);
    digitalWrite(SevenSegD, LOW);
    digitalWrite(SevenSegE, LOW);
    digitalWrite(SevenSegF, LOW);
    digitalWrite(SevenSegG, LOW);
}

void printNumCon()
{
    digitalWrite(SevenSegA, HIGH);
    digitalWrite(SevenSegD, HIGH);
    digitalWrite(SevenSegE, HIGH);
    digitalWrite(SevenSegF, HIGH);
}

void printNumCoff()
{
    digitalWrite(SevenSegA, LOW);
    digitalWrite(SevenSegD, LOW);
    digitalWrite(SevenSegE, LOW);
    digitalWrite(SevenSegF, LOW);
}

void printNumDon()
{
    digitalWrite(SevenSegB, HIGH);
    digitalWrite(SevenSegC, HIGH);
    digitalWrite(SevenSegD, HIGH);
    digitalWrite(SevenSegE, HIGH);
    digitalWrite(SevenSegG, HIGH);
}

void printNumDoff()
{
    digitalWrite(SevenSegB, LOW);
    digitalWrite(SevenSegC, LOW);
    digitalWrite(SevenSegD, LOW);
    digitalWrite(SevenSegE, LOW);
    digitalWrite(SevenSegG, LOW);
}

void printNumEon()
{
    digitalWrite(SevenSegA, HIGH);
    digitalWrite(SevenSegD, HIGH);
    digitalWrite(SevenSegE, HIGH);
    digitalWrite(SevenSegF, HIGH);
    digitalWrite(SevenSegG, HIGH);
}

void printNumEoff()
{
    digitalWrite(SevenSegA, LOW);
    digitalWrite(SevenSegD, LOW);
    digitalWrite(SevenSegE, LOW);
    digitalWrite(SevenSegF, LOW);
    digitalWrite(SevenSegG, LOW);
}

void printNumFon()
{
    digitalWrite(SevenSegA, HIGH);
    digitalWrite(SevenSegE, HIGH);
    digitalWrite(SevenSegF, HIGH);
    digitalWrite(SevenSegG, HIGH);
}

void printNumFoff()
{
    digitalWrite(SevenSegA, LOW);
    digitalWrite(SevenSegE, LOW);
    digitalWrite(SevenSegF, LOW);
    digitalWrite(SevenSegG, LOW);
}

void printNumLon()
{
    digitalWrite(SevenSegD, HIGH);
    digitalWrite(SevenSegE, HIGH);
    digitalWrite(SevenSegF, HIGH);
}

void printNumLoff()
{
    digitalWrite(SevenSegD, LOW);
    digitalWrite(SevenSegE, LOW);
    digitalWrite(SevenSegF, LOW);
}

void printNumRon()
{
    digitalWrite(SevenSegE, HIGH);
    digitalWrite(SevenSegG, HIGH);
}

void printNumRoff()
{
    digitalWrite(SevenSegE, LOW);
    digitalWrite(SevenSegG, LOW);
}

void printDPon()
{
    digitalWrite(SevenSegDP, HIGH);
}

void printDPoff()
{
    digitalWrite(SevenSegDP, LOW);
}

void printDashon()
{
    digitalWrite(SevenSegG, HIGH);
}

void printDashoff()
{
    digitalWrite(SevenSegG, LOW);
}