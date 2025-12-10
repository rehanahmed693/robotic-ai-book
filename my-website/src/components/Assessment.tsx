import React, { useState } from 'react';
import clsx from 'clsx';
import styles from './Assessment.module.css';

// Define types for assessment props
interface AssessmentProps {
  type: 'multiple-choice' | 'true-false' | 'short-answer' | 'practical';
  question: string;
  options?: string[]; // For multiple choice and true-false
  correctAnswer: string | number | boolean;
  explanation: string;
}

// Assessment component
const Assessment: React.FC<AssessmentProps> = ({ 
  type, 
  question, 
  options, 
  correctAnswer, 
  explanation 
}) => {
  const [userAnswer, setUserAnswer] = useState<string | boolean | null>(null);
  const [submitted, setSubmitted] = useState(false);
  const [isCorrect, setIsCorrect] = useState<boolean | null>(null);

  const handleSubmit = () => {
    let correct: boolean;
    
    if (type === 'true-false') {
      correct = userAnswer === correctAnswer;
    } else if (type === 'multiple-choice') {
      correct = userAnswer === String(correctAnswer);
    } else {
      // For short-answer and practical, we can't automatically check
      correct = true;
    }
    
    setIsCorrect(correct);
    setSubmitted(true);
  };

  const handleReset = () => {
    setUserAnswer(null);
    setSubmitted(false);
    setIsCorrect(null);
  };

  return (
    <div className={clsx('margin-vert--md', styles.assessment)}>
      <div className={styles.question}>
        <h4 className={styles.questionTitle}>Assessment Question</h4>
        <p className={styles.questionText}>{question}</p>
      </div>

      {type === 'multiple-choice' && options && (
        <div className={styles.answerOptions}>
          {options.map((option, index) => (
            <label key={index} className={styles.optionLabel}>
              <input
                type="radio"
                name="assessment-option"
                value={option}
                checked={userAnswer === option}
                onChange={(e) => setUserAnswer(e.target.value)}
                disabled={submitted}
                className={styles.optionInput}
              />
              <span className={styles.optionText}>{option}</span>
            </label>
          ))}
        </div>
      )}

      {type === 'true-false' && options && (
        <div className={styles.answerOptions}>
          {options.map((option, index) => (
            <label key={index} className={styles.optionLabel}>
              <input
                type="radio"
                name="assessment-truefalse"
                value={option}
                checked={userAnswer === option}
                onChange={(e) => setUserAnswer(e.target.value === 'True')}
                disabled={submitted}
                className={styles.optionInput}
              />
              <span className={styles.optionText}>{option}</span>
            </label>
          ))}
        </div>
      )}

      {(type === 'short-answer' || type === 'practical') && (
        <div className={styles.answerInput}>
          <textarea
            value={userAnswer as string || ''}
            onChange={(e) => setUserAnswer(e.target.value)}
            placeholder="Type your answer here..."
            disabled={submitted}
            className={styles.textArea}
          />
        </div>
      )}

      {!submitted ? (
        <button 
          onClick={handleSubmit}
          disabled={!userAnswer}
          className={clsx(styles.submitButton, !userAnswer && styles.submitButtonDisabled)}
        >
          Submit Answer
        </button>
      ) : (
        <div className={styles.result}>
          <div className={clsx(
            styles.feedback, 
            isCorrect ? styles.feedbackCorrect : styles.feedbackIncorrect
          )}>
            {isCorrect ? '✓ Correct!' : '✗ Try again'}
          </div>
          <div className={styles.explanation}>
            <strong>Explanation:</strong> {explanation}
          </div>
          <button 
            onClick={handleReset}
            className={styles.resetButton}
          >
            Try Again
          </button>
        </div>
      )}
    </div>
  );
};

export default Assessment;