import React from 'react';
import clsx from 'clsx';
import styles from './KeyTakeaways.module.css';

// Define the props interface
interface KeyTakeawaysProps {
  takeaways: string[];
}

// KeyTakeaways component
const KeyTakeaways: React.FC<KeyTakeawaysProps> = ({
  takeaways
}) => {
  if (!takeaways || takeaways.length === 0) {
    return null;
  }

  return (
    <div className={clsx(
      'margin-bottom--md',
      styles.keyTakeaways
    )}>
      <h3 className={styles.title}>Key Takeaways</h3>
      <ul className={styles.takeawaysList}>
        {takeaways.map((takeaway, index) => (
          <li key={index} className={styles.takeawayItem}>
            <span className={styles.icon}>💡</span>
            {takeaway}
          </li>
        ))}
      </ul>
    </div>
  );
};

export default KeyTakeaways;