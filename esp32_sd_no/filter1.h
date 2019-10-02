/******************************* SOURCE LICENSE *********************************
Copyright (c) 2019 MicroModeler.

A non-exclusive, nontransferable, perpetual, royalty-free license is granted to the Licensee to 
use the following Information for academic, non-profit, or government-sponsored research purposes.
Use of the following Information under this License is restricted to NON-COMMERCIAL PURPOSES ONLY.
Commercial use of the following Information requires a separately executed written license agreement.

This Information is distributed WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

******************************* END OF LICENSE *********************************/

// A commercial license for MicroModeler DSP can be obtained at http://www.micromodeler.com/launch.jsp

// Begin header file, filter1.h

static const int filter1_numStages = 4;
static const int filter1_coefficientLength = 20;
extern float filter1_coefficients[20];

typedef struct
{
  float state[16];
  float output;
} filter1Type;

typedef struct
{
  float *pInput;
  float *pOutput;
  float *pState;
  float *pCoefficients;
  short count;
} filter1_executionState;


filter1Type *filter1_create( void );
void filter1_destroy( filter1Type *pObject );
 void filter1_init( filter1Type * pThis );
 void filter1_reset( filter1Type * pThis );
#define filter1_writeInput( pThis, input )  filter1_filterBlock( pThis, &(input), &(pThis)->output, 1 );

#define filter1_readOutput( pThis )  (pThis)->output

 int filter1_filterBlock( filter1Type * pThis, float * pInput, float * pOutput, unsigned int count );
#define filter1_outputToFloat( output )  (output)

#define filter1_inputFromFloat( input )  (input)

 void filter1_filterBiquad( filter1_executionState * pExecState );

/******************************* SOURCE LICENSE *********************************
Copyright (c) 2019 MicroModeler.

A non-exclusive, nontransferable, perpetual, royalty-free license is granted to the Licensee to 
use the following Information for academic, non-profit, or government-sponsored research purposes.
Use of the following Information under this License is restricted to NON-COMMERCIAL PURPOSES ONLY.
Commercial use of the following Information requires a separately executed written license agreement.

This Information is distributed WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

******************************* END OF LICENSE *********************************/

// A commercial license for MicroModeler DSP can be obtained at http://www.micromodeler.com/launch.jsp

//#include "filter1.h"
//
//#include <stdlib.h> // For malloc/free
//#include <string.h> // For memset

float filter1_coefficients[20] = 
{
// Scaled for floating point

    0.9049501460290285, -1.7221672521853841, 0.9049501460290285, 1.8515194446102161, -0.9400005734362571,// b0, b1, b2, a1, a2
    1, -1.903052073909651, 1, 1.8841938118076462, -0.9663969965117779,// b0, b1, b2, a1, a2
    1, -1.903052073909651, 1, 1.8366925328018708, -0.936447369512947,// b0, b1, b2, a1, a2
    1, -1.903052073909651, 1, 1.851109411376213, -0.9612901011395015// b0, b1, b2, a1, a2

};

filter1Type *filter1_create( void )
{
  filter1Type *result = (filter1Type *)malloc( sizeof( filter1Type ) ); // Allocate memory for the object
  filter1_init( result );                     // Initialize it
  return result;                                // Return the result
}

void filter1_destroy( filter1Type *pObject )
{
  free( pObject );
}

 void filter1_init( filter1Type * pThis )
{
  filter1_reset( pThis );

}

 void filter1_reset( filter1Type * pThis )
{
  memset( &pThis->state, 0, sizeof( pThis->state ) ); // Reset state to 0
  pThis->output = 0;                  // Reset output

}

 int filter1_filterBlock( filter1Type * pThis, float * pInput, float * pOutput, unsigned int count )
{
  filter1_executionState executionState;          // The executionState structure holds call data, minimizing stack reads and writes 
  if( ! count ) return 0;                         // If there are no input samples, return immediately
  executionState.pInput = pInput;                 // Pointers to the input and output buffers that each call to filterBiquad() will use
  executionState.pOutput = pOutput;               // - pInput and pOutput can be equal, allowing reuse of the same memory.
  executionState.count = count;                   // The number of samples to be processed
  executionState.pState = pThis->state;                   // Pointer to the biquad's internal state and coefficients. 
  executionState.pCoefficients = filter1_coefficients;    // Each call to filterBiquad() will advance pState and pCoefficients to the next biquad

  // The 1st call to filter1_filterBiquad() reads from the caller supplied input buffer and writes to the output buffer.
  // The remaining calls to filterBiquad() recycle the same output buffer, so that multiple intermediate buffers are not required.

  filter1_filterBiquad( &executionState );    // Run biquad #0
  executionState.pInput = executionState.pOutput;         // The remaining biquads will now re-use the same output buffer.

  filter1_filterBiquad( &executionState );    // Run biquad #1

  filter1_filterBiquad( &executionState );    // Run biquad #2

  filter1_filterBiquad( &executionState );    // Run biquad #3

  // At this point, the caller-supplied output buffer will contain the filtered samples and the input buffer will contain the unmodified input samples.  
  return count;   // Return the number of samples processed, the same as the number of input samples

}

 void filter1_filterBiquad( filter1_executionState * pExecState )
{
  // Read state variables
  float w0, x0;
  float w1 = pExecState->pState[0];
  float w2 = pExecState->pState[1];

  // Read coefficients into work registers
  float b0 = *(pExecState->pCoefficients++);
  float b1 = *(pExecState->pCoefficients++);
  float b2 = *(pExecState->pCoefficients++);
  float a1 = *(pExecState->pCoefficients++);
  float a2 = *(pExecState->pCoefficients++);

  // Read source and target pointers
  float *pInput  = pExecState->pInput;
  float *pOutput = pExecState->pOutput;
  short count = pExecState->count;
  float accumulator;

  // Loop for all samples in the input buffer
  while( count-- )
  {
    // Read input sample
    x0 = *(pInput++);
  
    // Run feedback part of filter
    accumulator  = w2 * a2;
    accumulator += w1 * a1;
    accumulator += x0 ;

    w0 = accumulator ;
  
    // Run feedforward part of filter
    accumulator  = w0 * b0;
    accumulator += w1 * b1;
    accumulator += w2 * b2;

    w2 = w1;    // Shuffle history buffer
    w1 = w0;

    // Write output
    *(pOutput++) = accumulator ;
  }

  // Write state variables
  *(pExecState->pState++) = w1;
  *(pExecState->pState++) = w2;
}
