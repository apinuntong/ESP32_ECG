
static const int filter3_numStages = 4;
static const int filter3_coefficientLength = 20;
extern float filter3_coefficients[20];

typedef struct
{
  float state[16];
  float output;
} filter3Type;

typedef struct
{
  float *pInput;
  float *pOutput;
  float *pState;
  float *pCoefficients;
  short count;
} filter3_executionState;


filter3Type *filter3_create( void );
void filter3_destroy( filter3Type *pObject );
 void filter3_init( filter3Type * pThis );
 void filter3_reset( filter3Type * pThis );
#define filter3_writeInput( pThis, input ) filter3_filterBlock( pThis, &(input), &(pThis)->output, 1 );

#define filter3_readOutput( pThis ) (pThis)->output

 int filter3_filterBlock( filter3Type * pThis, float * pInput, float * pOutput, unsigned int count );
#define filter3_outputToFloat( output ) (output)

#define filter3_inputFromFloat( input ) (input)

 void filter3_filterBiquad( filter3_executionState * pExecState );

/******************************* SOURCE LICENSE *********************************
*/
float filter3_coefficients[20] = 
{
// Scaled for floating point

    0.9211709934999404, -1.4912218048754229, 0.9211709934999404, 1.5874228421025536, -0.9444884065311002,// b0, b1, b2, a1, a2
    1, -1.6188327850072708, 1, 1.6327533386384103, -0.9771915401573462,// b0, b1, b2, a1, a2
    1, -1.6188327850072708, 1, 1.5584626653330067, -0.9426687825148095,// b0, b1, b2, a1, a2
    1, -1.6188327850072708, 1, 1.5638067953246555, -0.9753153360030407// b0, b1, b2, a1, a2

};


filter3Type *filter3_create( void )
{
  filter3Type *result = (filter3Type *)malloc( sizeof( filter3Type ) ); // Allocate memory for the object
  filter3_init( result );                     // Initialize it
  return result;                                // Return the result
}

void filter3_destroy( filter3Type *pObject )
{
  free( pObject );
}

 void filter3_init( filter3Type * pThis )
{
  filter3_reset( pThis );

}

 void filter3_reset( filter3Type * pThis )
{
  memset( &pThis->state, 0, sizeof( pThis->state ) ); // Reset state to 0
  pThis->output = 0;                  // Reset output

}

 int filter3_filterBlock( filter3Type * pThis, float * pInput, float * pOutput, unsigned int count )
{
  filter3_executionState executionState;          // The executionState structure holds call data, minimizing stack reads and writes 
  if( ! count ) return 0;                         // If there are no input samples, return immediately
  executionState.pInput = pInput;                 // Pointers to the input and output buffers that each call to filterBiquad() will use
  executionState.pOutput = pOutput;               // - pInput and pOutput can be equal, allowing reuse of the same memory.
  executionState.count = count;                   // The number of samples to be processed
  executionState.pState = pThis->state;                   // Pointer to the biquad's internal state and coefficients. 
  executionState.pCoefficients = filter3_coefficients;    // Each call to filterBiquad() will advance pState and pCoefficients to the next biquad

  // The 1st call to filter1_filterBiquad() reads from the caller supplied input buffer and writes to the output buffer.
  // The remaining calls to filterBiquad() recycle the same output buffer, so that multiple intermediate buffers are not required.

  filter3_filterBiquad( &executionState );    // Run biquad #0
  executionState.pInput = executionState.pOutput;         // The remaining biquads will now re-use the same output buffer.

  filter3_filterBiquad( &executionState );    // Run biquad #1

  filter3_filterBiquad( &executionState );    // Run biquad #2

  filter3_filterBiquad( &executionState );    // Run biquad #3

  // At this point, the caller-supplied output buffer will contain the filtered samples and the input buffer will contain the unmodified input samples.  
  return count;   // Return the number of samples processed, the same as the number of input samples

}

 void filter3_filterBiquad( filter3_executionState * pExecState )
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
