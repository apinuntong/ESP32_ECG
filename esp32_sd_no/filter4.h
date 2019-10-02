
static const int filter4_numStages = 4;
static const int filter4_coefficientLength = 20;
extern float filter4_coefficients[20];

typedef struct
{
  float state[16];
  float output;
} filter4Type;

typedef struct
{
  float *pInput;
  float *pOutput;
  float *pState;
  float *pCoefficients;
  short count;
} filter4_executionState;


filter4Type *filter4_create( void );
void filter4_destroy( filter4Type *pObject );
 void filter4_init( filter4Type * pThis );
 void filter4_reset( filter4Type * pThis );
#define filter4_writeInput( pThis, input ) filter4_filterBlock( pThis, &(input), &(pThis)->output, 1 );

#define filter4_readOutput( pThis ) (pThis)->output

 int filter4_filterBlock( filter4Type * pThis, float * pInput, float * pOutput, unsigned int count );
#define filter4_outputToFloat( output )  (output)

#define filter4_inputFromFloat( input ) (input)

 void filter4_filterBiquad( filter4_executionState * pExecState );

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

float filter4_coefficients[20] = 
{
// Scaled for floating point

    0.9211709934999384, -0.5695960446974468, 0.9211709934999386, 0.6231147965916681, -0.9437926334351194,// b0, b1, b2, a1, a2
    1, -0.6183391017701264, 1.0000000000000002, 0.6651083890949481, -0.9764744111550878,// b0, b1, b2, a1, a2
    1, -0.6183391017701264, 1.0000000000000002, 0.5785065425330065, -0.9433637270969767,// b0, b1, b2, a1, a2
    1, -0.6183391017701264, 1.0000000000000002, 0.5558689349960189, -0.9760316137731497// b0, b1, b2, a1, a2

};


filter4Type *filter4_create( void )
{
  filter4Type *result = (filter4Type *)malloc( sizeof( filter4Type ) ); // Allocate memory for the object
  filter4_init( result );                     // Initialize it
  return result;                                // Return the result
}

void filter4_destroy( filter4Type *pObject )
{
  free( pObject );
}

 void filter4_init( filter4Type * pThis )
{
  filter4_reset( pThis );

}

 void filter4_reset( filter4Type * pThis )
{
  memset( &pThis->state, 0, sizeof( pThis->state ) ); // Reset state to 0
  pThis->output = 0;                  // Reset output

}

 int filter4_filterBlock( filter4Type * pThis, float * pInput, float * pOutput, unsigned int count )
{
  filter4_executionState executionState;          // The executionState structure holds call data, minimizing stack reads and writes 
  if( ! count ) return 0;                         // If there are no input samples, return immediately
  executionState.pInput = pInput;                 // Pointers to the input and output buffers that each call to filterBiquad() will use
  executionState.pOutput = pOutput;               // - pInput and pOutput can be equal, allowing reuse of the same memory.
  executionState.count = count;                   // The number of samples to be processed
  executionState.pState = pThis->state;                   // Pointer to the biquad's internal state and coefficients. 
  executionState.pCoefficients = filter4_coefficients;    // Each call to filterBiquad() will advance pState and pCoefficients to the next biquad

  // The 1st call to filter1_filterBiquad() reads from the caller supplied input buffer and writes to the output buffer.
  // The remaining calls to filterBiquad() recycle the same output buffer, so that multiple intermediate buffers are not required.

  filter4_filterBiquad( &executionState );    // Run biquad #0
  executionState.pInput = executionState.pOutput;         // The remaining biquads will now re-use the same output buffer.

  filter4_filterBiquad( &executionState );    // Run biquad #1

  filter4_filterBiquad( &executionState );    // Run biquad #2

  filter4_filterBiquad( &executionState );    // Run biquad #3

  // At this point, the caller-supplied output buffer will contain the filtered samples and the input buffer will contain the unmodified input samples.  
  return count;   // Return the number of samples processed, the same as the number of input samples

}

 void filter4_filterBiquad( filter4_executionState * pExecState )
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



  
